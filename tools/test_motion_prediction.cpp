#include "luckrobot_nav2_plugins/motion_prediction.hpp"
#include <cmath>
#include <iostream>
#include <stdexcept>

using namespace luckrobot_nav2_plugins::motion_prediction;
void check(bool ok, const char * name)
{
  if (!ok) {throw std::runtime_error(name);}
}
int main()
{
  int envelopes = 0;
  for (double a : {0.2, 0.35, 0.5}) {
    for (double lag : {0.0, 0.1, 0.25, 0.5}) {
      double previous = 0.0;
      for (int i = 0; i < 1000; ++i) {
        double d = i * 0.001;
        double v = stoppingSpeed(d, a, lag);
        check(std::isfinite(v) && v >= previous, "monotonic finite envelope");
        check(v * lag + v * v / (2 * a) <= d + 1e-10, "stopping distance");
        previous = v;
        ++envelopes;
      }
    }
  }
  check(stoppingSpeed(-1, .35, .25) == 0.0, "negative margin stops");
  check(stoppingSpeed(.01, .35, .25) < .04, "early terminal braking");
  check(advanceRate(.2, -.2, .2, .35, .1) > 0, "no instantaneous reversal");
  check(std::abs(advanceRate(.2, -.2, .2, .35, 1) + .0857142857) < 1e-8,
    "reversal splits braking and acceleration");
  check(advanceRate(.01, 0, .2, .35, .1) == 0, "no rate overshoot");
  check(advanceRate(-.01, 0, .2, .35, .1) == 0, "negative braking");
  for (double v : {0.03, 0.15, 0.30}) {
    for (double angle : {-.7, -.2, 0.0, .2, .7}) {
      for (double w : {-.35, -.1, 0.0, .1, .35}) {
        const double output = steering(v, angle, .16, w, .2, .08);
        const double nominal = 2 * v * std::sin(angle) / .16;
        check(std::abs(output - nominal) <= .080000001, "correction bounded");
        check(std::abs(output + steering(v, -angle, .16, -w, .2, .08)) < 1e-10,
          "left right symmetry");
        if (w == 0) {check(std::abs(output - nominal) < 1e-10, "no stationary bias");}
      }
    }
  }
  check(steering(.15, 0, .35, .25, .2, .08) < 0, "unwind before overshoot");
  check(steering(0, .5, .16, .25, .2, .08) == 0, "no unsolicited spin");
  check(trackingSpeedScale(.1, .3, -.3, .15, .45, .35) == 1,
    "small error keeps forward speed");
  check(trackingSpeedScale(.5, .25, .25, .15, .45, .35) == 1,
    "executed turn needs no lag slowdown");
  const double slow = trackingSpeedScale(.5, .25, -.25, .15, .45, .35);
  check(slow >= .35 && slow < .5, "wrong-direction execution slows, not stops");
  check(std::abs(slow - trackingSpeedScale(-.5, -.25, .25, .15, .45, .35)) < 1e-10,
    "lag slowdown symmetric");
  std::cout << "PASS " << envelopes << " braking envelopes, rate reversal and 75 steering cases\n";
}
