# Terminal creep refinement: run 194526

User clarified that only the final close approach feels slow. Keep the .75 m
approach range and .40/.80 speed caps unchanged. Do not globally increase speed
or shorten braking distance. Source predecessor 24c0b6d.

Bag analysis selects ordinary non-rotating tracking with abs carrot bearing<.15
and pose cost<100. At goal distance .08-.15 m (443 samples), command v median
.02276 m/s; .15-.25 m (265), median .05101; .25-.40 m median .10833.
Distances are Euclidean goal distances; controller uses remaining path too.
These are not a controlled timing trial and include existing safety scaling.

Add at most .02 m/s to the approach request, linearly blended only below
.25 m remaining distance. Bound by desired speed, terminal stopping envelope,
heading/curvature/cost regulation, trajectory check, smoother and guard as
before. At .15 m remaining with .08 m tolerance the added unregulated request
is .01176 m/s. No minimum-speed floor, goal tolerance relaxation or forcing
motion after arrival. At/beyond .25 m this term is zero. Physical test pending.

Six nonterminal rotation exits now occur at 4.04–4.37 deg with absolute
measured yaw .063–.098 rad/s, consistent with the preceding handoff fix.
Keep that logic rather than retuning it simultaneously.

Layer extraction: 372 camera-layer grids and 308 /map snapshots. Separate
final images show isolated occupied cells in /map as well as camera memory.
This does not establish which specific screenshot pixels are false obstacles,
nor prove repeated free-space observations at those cells. Do not erase small
clusters or static map occupancy solely on image appearance. Camera clearing
has improved per user report; static-map residual investigation remains open.
The raw-grid figure is saved locally as diagnostics/20261023_handoff/layers_194526.png.

Tests exercise small terminal boost, no effect outside terminal range, goal
hold, cost/brake limits and existing motion/controller/corner regressions.
Backup on Jetson: backups/20261024_terminal/luck_controller.cpp.
