# Cruise arc candidate withdrawn

User reported f8af594 felt worse than the prior version and did not noticeably
increase moving-turn speed. Restore luck_controller.cpp and its command tests
exactly from 9fe0799; retain analysis tools and research notes as history.
No configuration, planner, map clearing or in-place rotation changes belong
to f8af594. The only runtime change was curvature gain multiplied by
1-.15*clamp((remaining-approach_dist)/.25,0,1).

New evidence: custom_nav_20261024_221927.tar.gz, SHA256
683be239630df6f514446cb5318d9352cca28017938ca7f1cff03e7b4b284822.
Compared to 202805 selected cruise samples, curvature 1-2 command median
was .1721 versus .1602 m/s; curvature 2-4 .1093 versus .1053 m/s.
Instantaneous abs yaw command-feedback discrepancy p90 in curvature 2-4
was .4058 versus .1987 rad/s. Routes, samples and transient histories differ;
these observations neither establish causal degradation nor validate speedup.
User acceptance takes precedence over small changes in selected summary bins.

Rollback retains the earlier terminal approach improvement, alignment settling,
finite camera clearing, prior speed ceilings and all collision checks.
No motion or navigation launch was commanded during rollback.
