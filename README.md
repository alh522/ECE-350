# PID Path Following for an Underwater Robot (BlueBot in BlueSim)

**ECE 350-027 Introduction to Robotics, Lehigh University, Fall 2026: Final Project, Team 2**
Refia Temizkan and Alex Hume

---

## 1. What this project is about, in plain words

**BlueBot** is a small fish-like underwater robot. It has no propeller. It swims with fins:

| Fin | What it does |
|---|---|
| Caudal fin (tail) | Pushes the robot forward |
| Left and right pectoral fins (side fins) | Turn the robot. The left fin turns it left, the right fin turns it right |
| Dorsal fin (top) | Pushes it down (not used here: the robot stays at the surface) |

**BlueSim** is a physics simulator of BlueBot, provided by the course. It simulates how the fins move the robot through water, including drag. It does **not** come with any way to follow a path: there is no planned path, no measure of how far off the path the robot is, no controller and no performance numbers.

**Our goal:** make BlueBot follow a planned path (like a survey line a real underwater robot would drive) as closely as possible, and measure how well it does.

**Why it is hard:** each side fin can push with at most **0.006 N**, less than a third of the tail fin's 0.020 N (`dynamics.py`, lines 50-53). The side fins also push slightly backwards, so **every turn slows the robot down**. Sharp corners are therefore difficult.

The project has two parts:

- **Part I: PID trajectory control.** A PID controller steers the robot along a path. We compare it against a simpler P-only controller, on three paths and three levels of sensor noise.
- **Part II: Adaptive speed.** The robot slows down when it needs to turn hard and speeds up when it is lined up with the path. Does that improve tracking without making the run much slower?

---

## 2. How the controller works

Every control tick (about 0.2 s), the robot runs this loop:

```
  read pose  ->  heading error  ->   PID   ->  fin commands  ->  BlueSim moves the robot
 (x, y, angle)   to a point 300 mm    turn       left or right        (dynamics.update_ctrl)
  (+ optional     further along the   command u  pectoral fin,
   sensor noise)  path                           caudal = speed
       ^                                                                     |
       +---------------------------- repeat every tick ----------------------+
```

1. **Read where the robot is.** The position and heading come from the simulator's stored state (`environment.pos`), treated as a perfect sensor. Optionally we add random Gaussian noise to imitate a real, imperfect sensor.
2. **Find the closest point on the path**, then pick a target point **300 mm further along the path** (the "lookahead point").
3. **Heading error** = the angle the robot would have to turn to face that lookahead point. Steering toward a point ahead on the path is called *pure pursuit*.
4. **PID controller** turns the heading error `e` into a turn command `u`:

   `u = Kp * e + Ki * (integral of e) + Kd * (filtered rate of change of e)`

   - **P** (proportional): turn harder the bigger the error.
   - **I** (integral): correct an error that persists over time. Capped so it can't grow forever (anti-windup).
   - **D** (derivative): react to how fast the error is changing, which damps overshoot. It is smoothed with a low-pass filter so sensor noise doesn't make it jumpy.
5. **Fin commands.** If `u > 0`, the left side fin pushes (turn left); if `u < 0`, the right side fin pushes. Each is limited to 0..1. The tail fin sets the forward speed.
6. **BlueSim** integrates the physics for one tick and the loop repeats.

The **baseline** is a **P-only controller**: the same code with `Ki = Kd = 0` and its own tuned gain. Any difference between the two therefore comes from the I and D terms.

Note: the controller acts on the **heading error**. The **cross-track error** (distance from the path) is only used to score the run.

---

## 3. What we had to build (BlueSim doesn't provide it)

| Piece | What it does |
|---|---|
| Reference paths | `straight` (4 m line), `curve` (90 degree arc, 2.5 m radius), `zigzag` (three 1.4 m legs with two 90 degree corners) |
| Start pose | Every run starts 300 mm to the left of the path, turned 20 degrees away, so every run begins with a recovery |
| Cross-track error | Signed distance from the robot to the path (+ = left of the path). Only searches a short window ahead, so a robot that overshoots a corner can't "skip" a leg |
| Stop condition | A run counts as finished when the robot is within 150 mm of the last waypoint. Then all fins switch off (BlueSim never stops a robot on its own) |
| Sensor noise | `none`, `low` (10 mm, 2 degrees), `high` (30 mm, 6 degrees) standard deviation on the pose the controller reads |
| Metrics and logs | Every run writes its numbers to files in `logfiles/`, so every reported number can be traced to a run |

---

## 4. Files in this repository

| File | What it is |
|---|---|
| `pid_path_new.py` | **The main file.** The controller, the paths, the metrics, the logging, and the batch commands that run whole experiments. It is a BlueSim "behavior" file and must be placed in BlueSim's `fishfood/` folder to run (see Setup). Has a change log at the top |
| `pid_path.py` | The first version of the controller (kept for reference; `pid_path_new.py` replaces it) |
| `plot_ticks.py` | Optional helper that plots one or more runs: the robot's path, cross-track error, heading error and turn command over time. Not needed for the numbers; used for figures and for checking what happened in a run |
| `results/` | The summary CSV of every experiment (`gains`, `part1`, `dfilter`, `tune`, `part2`): the numbers behind every result below |
| `bluesim/` | Copy of simulator files |
| `README.md` | This file |

---

## 5. Setup

### 5.1 Requirements

- Python 3.9 or newer (tested on macOS with Python 3.9.6)
- Python packages: `numpy`, `scipy`, `matplotlib`
- Optional, for BlueSim's 3D animation: `ipyvolume`, `ipython_genutils`

```bash
python3 -m pip install numpy scipy matplotlib
python3 -m pip install ipyvolume ipython_genutils   # optional, animation only
```

Use `python3 -m pip` (not plain `pip`) so the packages go into the same Python that runs the simulator.

### 5.2 Get the simulator and put our files in place

1. Download the course simulator:
   ```bash
   git clone https://github.com/dilab-lehigh/Fall26_Lehigh_ECE250-027
   cd Fall26_Lehigh_ECE250-027
   ```
2. Copy our files into it:
   - `pid_path_new.py` into the `fishfood/` folder
   - `plot_ticks.py` into the main folder (next to `simulation.py`)

The folder should then look like this:
```
Fall26_Lehigh_ECE250-027/
    simulation.py        <- BlueSim's main script (run everything from this folder)
    dynamics.py          <- BlueSim physics
    environment.py
    animation.py
    plot_ticks.py        <- ours
    fishfood/
        pid_path_new.py  <- ours
    logfiles/            <- created automatically, all results go here
```

**Always run commands from the main `Fall26_Lehigh_ECE250-027` folder**, not from inside `fishfood/`.

Tip: if you keep this repository in its own folder inside the simulator folder (e.g. `Fall26_Lehigh_ECE250-027/ECE-350/`), link the file instead of copying it, so the version you edit is the version that runs:
```bash
ln -s ../ECE-350/pid_path_new.py fishfood/pid_path_new.py
```

---

## 6. How to run

### 6.1 One run

```bash
python3 -B simulation.py pid_path_new path=zigzag ctrl=pid speed=0.25 noise=none seed=1 tag=test
```

At the end it prints something like:
```
[pid_path] reached goal at 71.1 s, RMSE 105.3 mm (settled 65.3 mm)
```

`-B` stops Python from using old cached versions of the file (otherwise an edit can seem to have no effect).

Every setting after `pid_path_new` is optional. A typo in a setting name stops the run with an error, on purpose.

| Setting | Values | Meaning |
|---|---|---|
| `path` | `straight`, `curve`, `zigzag` | Which reference path |
| `ctrl` | `pid`, `p` | Full PID or the P-only baseline |
| `speed` | `0` to `1`, or `adaptive` | Tail-fin command (forward thrust), or the Part II adaptive rule |
| `noise` | `none`, `low`, `high` | Sensor noise level |
| `seed` | any integer | Random seed (changes the noise and the tick timing) |
| `tag` | any word | Label put in the logfile names, to keep experiments apart |
| `kp`, `ki`, `kd` | numbers | PID gains |
| `kp_p` | number | Gain of the P-only baseline |
| `alpha` | `0` < alpha <= `1` | Derivative filter. `1.0` = no filter, smaller = smoother but slower to react |
| `cmax`, `cmin`, `e0` | number, number, degrees | Part II: speed when lined up with the path, lowest speed, and the heading error where slowing down starts |

Current defaults (chosen by the tuning in section 8): `kp=4`, `ki=0.1`, `kd=2`, `kp_p=8`, `alpha=0.3`, `speed=0.25`; adaptive speed `cmax=0.3`, `cmin=0.05`, `e0=10`.

### 6.2 Whole experiments

These commands run many simulations one after another and collect the results:

```bash
python3 -B fishfood/pid_path_new.py gains     # tune the gains (tuning seeds 101-103 only)
python3 -B fishfood/pid_path_new.py part1     # Part I: P vs PID, 3 paths x 3 noise levels x 5 seeds = 90 runs
python3 -B fishfood/pid_path_new.py dfilter   # derivative filter on vs off, 30 runs
python3 -B fishfood/pid_path_new.py tune      # Part II: tune the adaptive speed rule, 54 runs
python3 -B fishfood/pid_path_new.py part2     # Part II: 6 fixed speeds vs adaptive, 3 paths x 3 noise x 5 seeds = 315 runs
```

Each simulation takes about a second, so even the largest batch finishes in a few minutes.

### 6.3 Summarize an experiment

```bash
python3 -B fishfood/pid_path_new.py report part1
```

This averages the runs that differ only in their seed and prints one line per setting, best (lowest RMSE) first, with the columns:

| Column | Meaning |
|---|---|
| `n` | Number of runs averaged |
| `reached` | Fraction of runs that reached the goal |
| `rmse` | Mean cross-track RMSE over the whole run [mm] |
| `settled` | Mean cross-track RMSE after the first 20 s [mm] |
| `time` | Mean time to reach the goal [s] |
| `chatter` | Mean change of the turn command per tick (0 = steady fins, 2 = flipping full left to full right every tick) |

### 6.4 Plot runs

`plot_ticks.py` overlays one or more runs: the robot paths from above with the reference path dashed, plus cross-track error, heading error, turn command and forward-speed command over time. It prints each run's time-weighted RMSE (matching the summary file). `out=` sets the file name inside `logfiles/`.

The two figures used in the slides:
```bash
# Part I: P-only vs PID on the zigzag
python3 plot_ticks.py logfiles/pidpath_part1_zigzag_p_0.25_low_*_s1_ticks.csv \
                      logfiles/pidpath_part1_zigzag_pid_0.25_low_*_s1_ticks.csv out=part1_zigzag.png
# Part II: fixed 0.25 and 0.3 vs adaptive on the zigzag
python3 plot_ticks.py logfiles/pidpath_part2_zigzag_pid_0.25_low_*_s1_ticks.csv \
                      logfiles/pidpath_part2_zigzag_pid_0.3_low_*_s1_ticks.csv \
                      logfiles/pidpath_part2_zigzag_pid_adaptive_low_*_s1_ticks.csv out=part2_zigzag.png
open logfiles/part2_zigzag.png        # macOS; on other systems open the file by hand
```

BlueSim's own 3D animation also works. Use the time stamp BlueSim prints at the end of a run, **plus `_0`** (the robot number):
```bash
python3 animation.py 261127_145303_0
open animations/261127_145303_0_animation.html
```

---

## 7. Outputs (`logfiles/`)

| File | Contents |
|---|---|
| `pidpath_<tag>_summary.csv` | One row per run: all settings plus `reached`, `completion_time_s`, `rmse_mm`, `settled_rmse_mm`, `max_cte_mm`, `path_crossings`, `mean_caudal`, `turn_change` (chatter), `min_goal_dist_mm`, `max_abs_z_mm` and the name of the run's ticks file. **New runs are appended**, so delete the file before re-running an experiment |
| `pidpath_<tag>_<path>_<ctrl>_<speed>_<noise>_kp.._kd.._a.._s<seed>_ticks.csv` | One row per control tick: time, x, y, z, heading, cross-track error, heading error, caudal command, left and right fin commands, reached flag |
| `<time>_0_data.txt`, `<time>_0_meta.txt` | BlueSim's own logs (used by `animation.py`) |

How the metrics are computed:
- **RMSE** = square root of the time-weighted mean of (cross-track error)^2, from the start until the goal is reached. Time-weighted because BlueSim's tick lengths vary by about 10%.
- **Settled RMSE** = the same, but only counting time after 20 s, so the start-up recovery (identical for every controller) doesn't dominate.
- Ticks after the goal are not counted: the robot keeps coasting after the fins switch off.

---

## 8. Results so far

All numbers below come from the summary files in `logfiles/` and can be reproduced with the commands in section 6.

### 8.1 Gain tuning (`gains`, zigzag, low noise, 3 tuning seeds per setting)

- Higher gains always lowered RMSE, but made the fins **chatter**: at the highest gains the fin command flipped between full left and full right nearly every tick on the straight legs, which wastes energy and would wear out a real robot.
- At the **same chatter level**, P-only and PID reached almost the same RMSE.
- We chose gains in the middle of the trade-off, at the same chatter (about 0.25) for both: **PID `kp=4, ki=0.1, kd=2`** and **P-only `kp_p=8`**.

### 8.2 Part I: PID vs P-only (`part1`, 5 seeds per cell, all 90 runs reached the goal)

Settled RMSE after 20 s, in mm (lower is better):

| Path | Noise | PID | P-only |
|---|---|---|---|
| Straight | none | **4.1** | 6.9 |
| Straight | low | **6.3** | 9.4 |
| Straight | high | **12.2** | 14.6 |
| Curve | none | **27.8** | 30.6 |
| Curve | low | **30.4** | 34.0 |
| Curve | high | **34.1** | 36.1 |
| Zigzag | none | 79.8 | **78.2** |
| Zigzag | low | 65.5 | **57.3** |
| Zigzag | high | 46.3 | **39.8** |

**What it means:**
- **PID wins on the smooth paths** (all 6 straight and curve conditions).
- **P-only wins at the sharp zigzag corners** (all 3). At the corners the side fins are already pushing as hard as they can, so the extra terms can't help, and the filtered D term adds a little delay.
- The **curve keeps about 30 mm of error for both** controllers. That comes from the lookahead method (aiming 300 mm ahead makes the robot cut inside an arc), not from the PID.

### 8.3 Derivative filter (`dfilter`, PID, zigzag, 5 seeds)

| Noise | No filter (`alpha=1.0`) | Filtered (`alpha=0.3`) |
|---|---|---|
| none | 108.6 mm, chatter 0.02 | 111.4 mm, chatter 0.02 |
| low | 97.0 mm, chatter 0.64 | 104.3 mm, chatter 0.25 |
| high | 104.4 mm, chatter 0.94 | **96.4 mm, chatter 0.59** |

The filter cuts chatter by 61% at low noise, and at high noise it is better on both RMSE and chatter. Cost: a few mm worse with no noise.

### 8.4 An unexpected finding

Adding sensor noise *lowered* RMSE (straight path: 97 mm with no noise vs 91 mm with high noise). The reason is speed: the noisy, chattering side fins push backwards and act as brakes (57 s vs 71 s to finish), and **a slower robot tracks better**. So RMSE must always be read together with completion time. This is accidental evidence for the Part II idea.

### 8.5 Part II: adaptive speed

**The rule.** The forward-speed command is `cmax` while the heading error is below `e0`, then falls in a straight line to `cmin` at 90 degrees. PID keeps steering; the rule only sets speed.

**Tuning (`tune`, zigzag, low noise, tuning seeds).** A first sweep with the top speed fixed at 1.0 failed badly (RMSE 278-622 mm; with `cmin=0.4` the robot never reached the goal): BlueBot has no brakes, so slowing down only after the heading error is already large comes too late at high speed. Making the top speed a setting (`cmax`, change 7) fixed this. Lower top speed always tracked better but finished later; `cmax=0.3, cmin=0.05, e0=10` was the best balance.

**Results (`part2`, 5 new seeds per cell).** Cross-track RMSE and time to finish, zigzag:

| Noise | Fixed 0.25 | Fixed 0.3 | Fixed 0.35 | **Adaptive** |
|---|---|---|---|---|
| none | 111.4 mm, 68.1 s | 157.2 mm, 64.5 s | 249.4 mm, 63.3 s | **103.3 mm, 67.6 s** |
| low | 104.3 mm, 70.0 s | 143.9 mm, 65.0 s | 238.9 mm, 63.6 s | **99.3 mm, 68.9 s** |
| high | 96.4 mm, 74.8 s | 133.2 mm, 67.6 s | 223.0 mm, 64.4 s | **94.7 mm, 73.0 s** |

Fixed 0.5 and 0.75 never reached the goal on the zigzag; fixed 1.0 reached it only by swinging wide (RMSE about 660 mm).

- **On the zigzag, adaptive beats every fixed speed:** lower RMSE *and* faster than the best fixed speed (0.25) at all three noise levels, with 17-23% lower settled error. This meets the proposal's success test.
- **On the straight line and the gentle curve it brings nothing:** it behaves like a fixed speed of 0.3 and is slightly worse (about 2 mm) and slightly slower (1-2 s), because it only slows during the start-up recovery.
- **Conclusion:** slowing down for large heading errors helps exactly where turns are hard, as long as the top speed is modest enough that the robot can still react before a corner.

---

## 9. Common problems

| Problem | Fix |
|---|---|
| `ModuleNotFoundError: No module named 'numpy'` (or scipy) | `python3 -m pip install numpy scipy matplotlib`, and run with `python3`, not `python` |
| `can't open file ... simulation.py` | You are in the wrong folder. `cd` into `Fall26_Lehigh_ECE250-027` |
| An edit to the code has no effect | Make sure you edited the copy in `fishfood/` (or use the link in section 5.2), save the file, and run with `-B` |
| `report` shows duplicate or misaligned rows | The summary file appends. Delete `logfiles/pidpath_<tag>_summary.csv` and re-run the experiment |
| `animation.py` says the data file does not exist | Add `_0` to the time stamp: `python3 animation.py 261127_145303_0` |
| The robot keeps moving after the goal | Expected: BlueSim has no brakes. Fins switch off at the goal and the robot coasts; those ticks are not counted |

---

## 10. Limitations

- Position and heading come from the simulator's stored state (a perfect sensor plus added noise), not from BlueBot's cameras.
- One robot, 2D only (the robot stays at the surface).
- Gains and the adaptive rule were tuned on one path (zigzag, low noise) and used everywhere.
- The adaptive rule reacts to heading error; it does not look ahead to slow down *before* a corner.
- The lookahead distance (300 mm) was fixed, not tuned.
- Tick timing in BlueSim varies by about 10% and is not logged by BlueSim itself; our metrics weight by the measured tick length.

---

## 11. AI use

As required by the course, AI-generated code is marked in the source with `# AI-GENERATED (...)` and `# END AI-GENERATED` comments, and every substantive use is listed in the paper's AI appendix.

- `pid_path.py` (version 1): generated with Claude Fable (appendix entry 1).
- `plot_ticks.py`: written with Claude Opus 5.5.

---

## 12. Who did what

*To be filled in by the team.*

---

## References

1. D. Ni, *ECE 350-027 Introduction to Robotics: Robot Control, PID Control*, Lehigh University.
2. F. Berlinger, P. Wulkop and R. Nagpal, "Self-Organized Evasive Fountain Maneuvers with a Bioinspired Underwater Robot Collective," *IEEE ICRA*, 2021, pp. 9204-9211. (The model behind BlueSim's `dynamics.py`.)
3. R. C. Coulter, "Implementation of the Pure Pursuit Path Tracking Algorithm," Carnegie Mellon University Robotics Institute, Tech. Rep. CMU-RI-TR-92-01, 1992.
