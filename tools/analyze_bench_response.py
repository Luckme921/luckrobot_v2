#!/usr/bin/env python3
"""Offline bench telemetry summary, raw protocol RPM signs retained."""
import json
import statistics as s
import sys
d = json.load(open(sys.argv[1]))
rows = d['rows']
drivers = [r for r in rows if r[0] == 'driver']
cmds = [r for r in rows if r[0] == 'cmd']
print('status:', d['status'], 'driver samples:', len(drivers))
print('bus mean/max seconds:', s.mean(r[13] for r in drivers), max(r[13] for r in drivers))
print('bad replies:', sum(r[11] != 1 or r[12] != 1 for r in drivers))
print('watchdog samples:', sum(r[14] != 0 for r in drivers))
motors = [r for r in rows if r[0] == 'motor']
if motors:
    print('motor faults:', sorted(set(r[5] for r in motors)),
          'modes:', sorted(set(r[3] for r in motors)),
          'max_abs_current_A:', max(abs(r[4]) for r in motors))
previous = (0,0)
for i,c in enumerate(cmds):
    current = tuple(c[2:4])
    if current == previous:
        continue
    previous = current
    if current == (0,0):
        continue
    start = c[1]
    end = next(r[1] for r in cmds[i+1:] if tuple(r[2:4]) != current)
    part = [r for r in drivers if start <= r[1] < end]
    tail = [r for r in part if r[1] > end-.5]
    print('\ncommand', current, 'start', round(start,3))
    for name,ci,fi,ok in [('left',7,9,11),('right',8,10,12)]:
        target = s.median(r[ci] for r in tail)
        sent = next((r[1]-start for r in part if r[ci] == target), None)
        hit = next((r[1]-start for r in part if r[ok] == 1 and r[fi]*target >= .9*target*target), None)
        print(name, 'target', target, 'sent_at', sent, 'first90_at', hit,
              'last_half_second_feedback_mean', s.mean(r[fi] for r in tail if r[ok] == 1))
