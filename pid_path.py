# AI-GENERATED ([tool/model], [date]) -- appendix entry [N]
"""
pid_path.py -- PID path following for ONE BlueBot (Part I) plus adaptive
forward speed (Part II).

WHAT THIS FILE DOES
  1. Builds a reference path (a list of x,y points) inside the tank.
  2. Every control tick it reads the robot pose from environment.pos (treated
     as an ideal sensor, optionally with added noise), finds a "lookahead"
     point 300 mm further along the path, and computes the heading error
     toward that point.
  3. A PID controller (or P-only baseline) turns the heading error into a
     left / right pectoral-fin command. The caudal fin gives forward thrust,
     either at a fixed level or slowed down when the heading error is large
     (adaptive speed, Part II).
  4. The commands go to dynamics.update_ctrl(), so the controller really
     drives the simulated robot.
  5. It measures cross-track error from the TRUE pose, and writes the results
     to ./logfiles/ so every number in the paper traces back to a run.

HOW TO RUN (always from the repo root, the folder with simulation.py)
  One run (every setting is optional, defaults are in DEFAULTS below):
    python -B simulation.py pid_path path=zigzag ctrl=pid speed=0.25 noise=none seed=1 tag=test
  Whole experiments (these just call the line above many times):
    python -B fishfood/pid_path.py part1    # Part I : P vs PID, all paths and noise levels
    python -B fishfood/pid_path.py part2    # Part II: fixed speeds vs adaptive speed
    python -B fishfood/pid_path.py tune     # tune c_min and e0 on separate seeds

OUTPUT (all in ./logfiles/)
  pidpath_<tag>_summary.csv : one row per run (settings + metrics). Appended.
  pidpath_<tag>_<path>_<ctrl>_<speed>_<noise>_s<seed>_ticks.csv :
                              one row per control tick (for trajectory plots).
  BlueSim also writes its own <time>_0_data.txt / _meta.txt as usual.

UNITS
  position x, y, z ........ millimetres (same as environment.pos), z = 0 is the surface
  heading phi, angles ..... radians, counter-clockwise from the +x axis
                            (settings typed on the command line are in degrees)
  time .................... seconds
  fin commands ............ unitless, 0 to 1 (1 = maximum fin thrust)
  (dynamics.py converts to metres internally; this file never needs metres.)
"""
import math
import os
import random
import subprocess
import sys
import time

import numpy as np

EXPERIMENT_NAME = 'PID path following (single robot)'

# ----------------------------------------------------------------------------
# SETTINGS
# ----------------------------------------------------------------------------
# Every key here can be overridden on the command line as key=value.
DEFAULTS = {
    'path': 'zigzag',    # straight | curve | zigzag
    'ctrl': 'pid',       # pid | p   (p = P-only baseline: same kp, ki = kd = 0)
    'speed': '0.25',     # caudal command 0..1, or the word "adaptive"
    'noise': 'none',     # none | low | high   (see NOISE_LEVELS)
    'seed': '1',         # random seed for sensor noise and tick-length jitter
    'tag': 'test',       # label that goes into the logfile names
    'kp': '2.0',         # proportional gain [fin command per rad of heading error]
    'ki': '0.1',         # integral gain     [fin command per rad*s]
    'kd': '1.0',         # derivative gain   [fin command per rad/s]
    'cmin': '0.2',       # adaptive speed: lowest caudal command
    'e0': '20',          # adaptive speed: heading error [deg] where slowing begins
}

# Noise added to the pose the CONTROLLER reads: (position std [mm], heading std [deg]).
NOISE_LEVELS = {'none': (0.0, 0.0), 'low': (10.0, 2.0), 'high': (30.0, 6.0)}

LOOKAHEAD = 300.0          # [mm] how far ahead on the path the robot aims
GOAL_RADIUS = 150.0        # [mm] the run counts as finished inside this distance
START_OFFSET = 300.0       # [mm] start this far to the LEFT of the path start
START_HEADING_OFF = math.radians(20)  # [rad] start heading turned 20 deg left of the path
ADAPTIVE_CMAX = 1.0        # caudal command of the adaptive rule when well aligned
ADAPTIVE_E_FULL = math.radians(90)    # [rad] heading error where the slow-down reaches cmin
INTEGRAL_LIMIT = 1.0       # [rad*s] cap on the stored integral (anti-windup)
SEARCH_WINDOW = 200.0      # [mm] how far ahead of the last progress we look for the closest
                           # path point (top speed is about 32 mm per tick, so this is ample)

CLOCK_FREQ = 5             # [Hz] must match clock_freq in simulation.py
Simulation_time = 120      # [s] read by simulation.py: time limit of one run
N_fish = 1                 # read by simulation.py: we simulate a single robot
TOTAL_TICKS = N_fish * Simulation_time * CLOCK_FREQ  # run() is called exactly this often


def read_settings():
    """Start from DEFAULTS and overwrite with key=value words from the command line.

    simulation.py only looks at sys.argv[1] (the behavior name), so everything
    after it is free for us. This is how we choose conditions without editing
    simulation.py.
    """
    settings = dict(DEFAULTS)
    for word in sys.argv[2:]:
        if '=' not in word:
            continue
        key, value = word.split('=', 1)
        if key not in settings:
            # Stop on typos: a silently ignored setting would give a wrong logfile.
            print('pid_path: unknown setting "{}". Allowed: {}'.format(key, list(DEFAULTS)))
            sys.exit()
        settings[key] = value
    return settings


SETTINGS = read_settings()


# ----------------------------------------------------------------------------
# REFERENCE PATH
# ----------------------------------------------------------------------------
def make_path(name):
    """Return the reference path as an array of [x, y] points in mm.

    All three paths are about 4 m long and stay within 2.8 m of the tank
    centre, because BlueSim clamps a robot that touches the wall (r = 3 m).
    """
    if name == 'straight':
        points = [[-2000, 0], [2000, 0]]
    elif name == 'curve':
        # Quarter circle (90 deg) of radius 2.5 m around the tank centre,
        # approximated by 30 short straight pieces (about 131 mm each).
        angles = np.linspace(math.radians(-135), math.radians(-45), 31)
        points = [[2500 * math.cos(a), 2500 * math.sin(a)] for a in angles]
    elif name == 'zigzag':
        # Z shape: three 1.4 m legs joined by two 90 deg corners (left, then right).
        points = [[-1400, -700], [0, -700], [0, 700], [1400, 700]]
    else:
        print('pid_path: unknown path "{}"'.format(name))
        sys.exit()
    return np.array(points, dtype=float)


def cumulative_length(path):
    """Distance along the path from its start to each path point [mm]."""
    steps = np.hypot(np.diff(path[:, 0]), np.diff(path[:, 1]))
    return np.concatenate(([0.0], np.cumsum(steps)))


def start_pose(path):
    """Start pose [x, y, z, phi]: beside the path start and turned away from it.

    Starting exactly on the path would make the straight line trivial, so the
    robot starts START_OFFSET to the left and START_HEADING_OFF turned further
    left. That way every path begins with a recovery.
    """
    direction = math.atan2(path[1, 1] - path[0, 1], path[1, 0] - path[0, 0])
    left = direction + math.pi / 2  # 90 deg counter-clockwise = "left" of travel
    x = path[0, 0] + START_OFFSET * math.cos(left)
    y = path[0, 1] + START_OFFSET * math.sin(left)
    return [x, y, 0.0, direction + START_HEADING_OFF]


PATH = make_path(SETTINGS['path'])
PATH_LEN = cumulative_length(PATH)
Leader_initial = start_pose(PATH)  # read by simulation.py: start pose of robot 0


# ----------------------------------------------------------------------------
# TRACKING-ERROR GEOMETRY
# ----------------------------------------------------------------------------
def wrap_angle(angle):
    """Wrap an angle to [-pi, pi]. BlueSim never wraps phi, so errors must be."""
    return math.atan2(math.sin(angle), math.cos(angle))


def closest_on_path(x, y, s_from):
    """Find the closest path point to (x, y), searching only a short window ahead.

    s_from [mm] is how far along the path the robot already was on the previous
    tick. We only search from s_from to s_from + SEARCH_WINDOW, so progress can
    never go backwards or jump. WHY: a robot that overshoots a corner can end up
    nearer to a LATER leg of the path; a search over the whole path would then
    let it skip a leg and still report a small error.

    Returns (cross_track, s):
      cross_track [mm] : distance to that point, positive when the robot is LEFT of the path
      s [mm]           : how far along the path that closest point is
    """
    s_to = s_from + SEARCH_WINDOW
    best_dist = float('inf')
    best_cross = 0.0
    best_s = s_from
    for i in range(len(PATH) - 1):
        ax, ay = PATH[i]
        bx, by = PATH[i + 1]
        seg_len = PATH_LEN[i + 1] - PATH_LEN[i]
        # t runs from 0 (start of this straight piece) to 1 (its end).
        # t_low..t_high is the part of the piece that lies inside the search window.
        t_low = max(0.0, (s_from - PATH_LEN[i]) / seg_len)
        t_high = min(1.0, (s_to - PATH_LEN[i]) / seg_len)
        if t_low > t_high:
            continue  # this piece is completely outside the window
        # projection of the robot onto the piece, kept inside the allowed part
        t = ((x - ax) * (bx - ax) + (y - ay) * (by - ay)) / seg_len ** 2
        t = min(t_high, max(t_low, t))
        qx = ax + t * (bx - ax)
        qy = ay + t * (by - ay)
        dist = math.hypot(x - qx, y - qy)
        if dist < best_dist:
            best_dist = dist
            # z-part of the cross product: > 0 means the robot is left of the piece
            side = (bx - ax) * (y - ay) - (by - ay) * (x - ax)
            best_cross = dist if side >= 0 else -dist
            best_s = PATH_LEN[i] + t * seg_len
    return best_cross, best_s


def point_at(s):
    """The [x, y] point that lies s mm along the path (held at the end beyond it)."""
    x = np.interp(s, PATH_LEN, PATH[:, 0])
    y = np.interp(s, PATH_LEN, PATH[:, 1])
    return x, y


# ----------------------------------------------------------------------------
# THE ROBOT
# ----------------------------------------------------------------------------
class Fish():
    """One BlueBot that follows PATH."""

    def __init__(self, my_id, dynamics, environment):
        self.id = my_id
        self.dynamics = dynamics
        self.environment = environment

        # --- settings for this run, converted from command-line text ---
        self.kp = float(SETTINGS['kp'])
        self.ki = float(SETTINGS['ki'])
        self.kd = float(SETTINGS['kd'])
        if SETTINGS['ctrl'] == 'p':
            # Baseline: identical controller with the I and D terms switched off,
            # so any difference in the results comes from those two terms only.
            self.ki = 0.0
            self.kd = 0.0
        self.adaptive = (SETTINGS['speed'] == 'adaptive')
        self.fixed_caudal = 0.0 if self.adaptive else float(SETTINGS['speed'])
        self.cmin = float(SETTINGS['cmin'])
        self.e0 = math.radians(float(SETTINGS['e0']))
        self.noise_pos, noise_deg = NOISE_LEVELS[SETTINGS['noise']]
        self.noise_phi = math.radians(noise_deg)

        # simulation.py hard-codes its seeds, so every run would be identical.
        # Re-seeding here (after simulation.py seeded) makes seed=N change both
        # the tick-length jitter (random) and our sensor noise (np.random).
        seed = int(SETTINGS['seed'])
        random.seed(seed)
        np.random.seed(seed)

        # --- controller memory ---
        self.integral = 0.0     # running sum of heading error * dt [rad*s]
        self.prev_error = None  # heading error of the previous tick [rad]
        self.s_measured = 0.0   # progress along the path as the controller sees it [mm]
        self.s_true = 0.0       # progress along the path from the true pose [mm]

        # --- bookkeeping for the metrics ---
        self.t = 0.0                 # time since the start [s] (sum of tick lengths)
        self.ticks = 0               # number of control ticks done
        self.reached = False         # True once inside GOAL_RADIUS
        self.completion_time = float('nan')
        self.sum_cte2_dt = 0.0       # sum of cross_track^2 * dt, for the RMSE
        self.sum_dt = 0.0            # time over which the RMSE is taken
        self.max_cte = 0.0           # largest |cross_track| seen [mm]
        self.crossings = 0           # times the robot crossed the path (oscillation count)
        self.prev_cte = None
        self.sum_caudal_dt = 0.0     # for the mean caudal command
        self.max_abs_z = 0.0         # to prove the robot stayed at the surface
        self.min_goal_dist = float('inf')  # closest the robot ever got to the final waypoint [mm]
        self.rows = []               # one row per tick, written to the ticks file

    def run(self, duration):
        """Called by simulation.py once per control tick (about 0.2 s, jittered).

        We do not call environment.get_robots(): a single robot has no
        neighbours, and the shipped "if not robots: return" guard would skip
        the fin command entirely (handout trap).
        """
        target_pos, vel = self.move(duration)
        self.environment.update_states(self.id, target_pos, vel)
        self.t += duration
        self.ticks += 1
        self.max_abs_z = max(self.max_abs_z, abs(self.environment.pos[self.id, 2]))
        if self.ticks == TOTAL_TICKS:
            # simulation.py cannot be stopped from here and has no "end" hook,
            # so we write our logs on the tick we know to be the last one.
            self.write_logs()

    def measure_pose(self, x, y, phi):
        """The pose the CONTROLLER sees: true pose plus zero-mean Gaussian noise."""
        if self.noise_pos > 0:
            x = x + np.random.normal(0, self.noise_pos)
            y = y + np.random.normal(0, self.noise_pos)
            phi = phi + np.random.normal(0, self.noise_phi)
        return x, y, phi

    def heading_error(self, x, y, phi):
        """Angle [rad] the robot must turn to face the lookahead point (+ = turn left)."""
        cross, self.s_measured = closest_on_path(x, y, self.s_measured)
        look_x, look_y = point_at(self.s_measured + LOOKAHEAD)
        return wrap_angle(math.atan2(look_y - y, look_x - x) - phi)

    def pid(self, error, dt):
        """PID on the heading error. Returns the turn command u (+ = turn left)."""
        self.integral += error * dt
        # Anti-windup: during a long saturated turn the sum would keep growing
        # and push the robot past the path afterwards, so we cap it.
        self.integral = min(INTEGRAL_LIMIT, max(-INTEGRAL_LIMIT, self.integral))
        if self.prev_error is None:
            derivative = 0.0  # no previous sample on the first tick
        else:
            # use the real (jittered) tick length, not a nominal 0.2 s
            derivative = wrap_angle(error - self.prev_error) / dt
        self.prev_error = error
        return self.kp * error + self.ki * self.integral + self.kd * derivative

    def caudal_command(self, error):
        """Forward thrust command. Fixed, or reduced when the heading error is large."""
        if not self.adaptive:
            return self.fixed_caudal
        size = abs(error)
        if size <= self.e0:
            return ADAPTIVE_CMAX
        # straight line from CMAX at e0 down to cmin at 90 deg, then flat at cmin
        fraction = min(1.0, (size - self.e0) / (ADAPTIVE_E_FULL - self.e0))
        return ADAPTIVE_CMAX - fraction * (ADAPTIVE_CMAX - self.cmin)

    def move(self, duration):
        """One control tick: sense, decide, command the fins, integrate the dynamics."""
        # TRUE pose: used only for the metrics and the stop condition.
        x, y, z, phi = self.environment.pos[self.id]
        cte, self.s_true = closest_on_path(x, y, self.s_true)
        goal_dist = math.hypot(PATH[-1, 0] - x, PATH[-1, 1] - y)
        self.min_goal_dist = min(self.min_goal_dist, goal_dist)

        if not self.reached and goal_dist < GOAL_RADIUS:
            self.reached = True
            self.completion_time = self.t  # first moment inside the goal circle

        if self.reached:
            # Nothing in BlueSim stops on its own: switch all fins off.
            # The robot keeps coasting (only drag slows it), so metrics are
            # frozen from here on and completion_time is already recorded.
            error = 0.0
            caudal = 0.0
            pect_l = 0.0
            pect_r = 0.0
        else:
            mx, my, mphi = self.measure_pose(x, y, phi)
            error = self.heading_error(mx, my, mphi)
            u = self.pid(error, duration)
            # In dynamics.py the LEFT pectoral fin increases phi (turn left) and the
            # RIGHT one decreases it. A fin cannot push backwards, so only one fin
            # works at a time, and update_ctrl() does not limit its inputs, so we
            # limit to 0..1 here.
            pect_l = min(1.0, max(0.0, u))
            pect_r = min(1.0, max(0.0, -u))
            caudal = self.caudal_command(error)
            self.update_metrics(cte, caudal, duration)

        # dorsal = 0: buoyancy keeps the robot at the surface (z = 0), so the task is 2D.
        self.dynamics.update_ctrl(0.0, caudal, pect_r, pect_l)
        self.rows.append([self.t, x, y, z, phi, cte, error, caudal, pect_l, pect_r, int(self.reached)])
        return self.dynamics.simulate_move(self.id, duration)

    def update_metrics(self, cte, caudal, dt):
        """Add this tick to the running metrics (only called before the goal is reached)."""
        # Weight by dt because tick lengths are jittered (about 10 %).
        self.sum_cte2_dt += cte ** 2 * dt
        self.sum_caudal_dt += caudal * dt
        self.sum_dt += dt
        self.max_cte = max(self.max_cte, abs(cte))
        if self.prev_cte is not None and cte * self.prev_cte < 0:
            self.crossings += 1  # sign change = the robot crossed the path
        self.prev_cte = cte

    def write_logs(self):
        """Write the per-tick file and append one summary row for this run."""
        os.makedirs('logfiles', exist_ok=True)  # simulation.py creates it only afterwards
        s = SETTINGS
        rmse = math.sqrt(self.sum_cte2_dt / self.sum_dt)
        mean_caudal = self.sum_caudal_dt / self.sum_dt

        run_name = 'pidpath_{}_{}_{}_{}_{}_s{}'.format(
            s['tag'], s['path'], s['ctrl'], s['speed'], s['noise'], s['seed'])
        np.savetxt('logfiles/' + run_name + '_ticks.csv', np.array(self.rows), fmt='%.4f', delimiter=',',
                   header='t_s,x_mm,y_mm,z_mm,phi_rad,cross_track_mm,heading_error_rad,'
                          'caudal,pect_l,pect_r,reached', comments='')

        header = ('timestamp,tag,path,ctrl,speed,noise,seed,kp,ki,kd,cmin,e0_deg,lookahead_mm,'
                  'reached,completion_time_s,rmse_mm,max_cte_mm,path_crossings,mean_caudal,'
                  'min_goal_dist_mm,max_abs_z_mm,ticks_file')
        values = [time.strftime('%Y-%m-%d %H:%M:%S'), s['tag'], s['path'], s['ctrl'], s['speed'],
                  s['noise'], s['seed'], self.kp, self.ki, self.kd, self.cmin, s['e0'], LOOKAHEAD,
                  int(self.reached), round(self.completion_time, 2), round(rmse, 2),
                  round(self.max_cte, 2), self.crossings, round(mean_caudal, 3),
                  round(self.min_goal_dist, 1), round(self.max_abs_z, 3), run_name + '_ticks.csv']
        summary_file = 'logfiles/pidpath_{}_summary.csv'.format(s['tag'])
        is_new = not os.path.exists(summary_file)
        with open(summary_file, 'a') as f:
            if is_new:
                f.write(header + '\n')
            f.write(','.join(str(v) for v in values) + '\n')
        if self.reached:
            print('\n[pid_path] reached goal at {:.1f} s, RMSE {:.1f} mm'.format(self.completion_time, rmse))
        else:
            print('\n[pid_path] NOT reached (closest approach {:.0f} mm), RMSE {:.1f} mm'.format(self.min_goal_dist, rmse))


# ----------------------------------------------------------------------------
# BATCH RUNS  (only used when this file is started directly, see header)
# ----------------------------------------------------------------------------
# simulation.py is a script that runs one experiment and exits, so the simplest
# way to run many conditions without editing it is to start it again for each
# one. subprocess.call does exactly that and waits until the run is finished.
PATHS = ['straight', 'curve', 'zigzag']
NOISES = ['none', 'low', 'high']
REPORT_SEEDS = [1, 2, 3, 4, 5]     # seeds behind the numbers in the paper
TUNE_SEEDS = [101, 102, 103]       # separate seeds, used only for tuning
PART1_SPEED = 0.25                 # fixed caudal command for Part I (see note in batch_part1)


def run_one(tag, path, ctrl, speed, noise, seed, extra=()):
    """Start one simulation with these settings and wait for it to finish."""
    module = os.path.splitext(os.path.basename(__file__))[0]  # "pid_path"
    command = [sys.executable, '-B', 'simulation.py', module,
               'tag=' + tag, 'path=' + path, 'ctrl=' + ctrl, 'speed=' + str(speed),
               'noise=' + noise, 'seed=' + str(seed)] + list(extra)
    print('>>>', ' '.join(command))
    subprocess.call(command)


def batch_part1():
    """Part I: P-only vs PID at one fixed speed. 3 paths x 2 controllers x 3 noise x 5 seeds = 90 runs.

    PART1_SPEED is 0.25 because in test runs at 0.5 the robot could not take the
    90 deg corners of the zigzag (its turning circle is wider than the lookahead
    distance, so it orbits the corner and never arrives) with either controller.
    """
    for path in PATHS:
        for ctrl in ['p', 'pid']:
            for noise in NOISES:
                for seed in REPORT_SEEDS:
                    run_one('part1', path, ctrl, PART1_SPEED, noise, seed)


def batch_part2():
    """Part II: PID at 4 fixed speeds vs adaptive speed. 3 paths x 5 speeds x 3 noise x 5 seeds = 225 runs."""
    for path in PATHS:
        for speed in [0.25, 0.5, 0.75, 1.0, 'adaptive']:
            for noise in NOISES:
                for seed in REPORT_SEEDS:
                    run_one('part2', path, 'pid', speed, noise, seed)


def batch_tune():
    """Tune the adaptive rule on the corner path with TUNE_SEEDS. 3 x 3 x 3 seeds = 27 runs."""
    for cmin in [0.1, 0.2, 0.4]:
        for e0 in [10, 20, 40]:
            for seed in TUNE_SEEDS:
                run_one('tune', 'zigzag', 'pid', 'adaptive', 'low', seed,
                        extra=['cmin=' + str(cmin), 'e0=' + str(e0)])


if __name__ == '__main__':
    batches = {'part1': batch_part1, 'part2': batch_part2, 'tune': batch_tune}
    if len(sys.argv) < 2 or sys.argv[1] not in batches:
        print('Usage (from the repo root): python -B fishfood/pid_path.py part1|part2|tune')
        sys.exit()
    batches[sys.argv[1]]()
# END AI-GENERATED
