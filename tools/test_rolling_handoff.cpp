#include "explore/rolling_handoff.hpp"
#include <cassert>
#include <limits>
#include <iostream>
int main() {
 using explore::rollingSuccessorSafe;
 assert(rollingSuccessorSafe(.25,0,1.2,0,.2,.52));
 assert(rollingSuccessorSafe(.25,0,1.0,.2,.2,.52));
 assert(!rollingSuccessorSafe(.25,0,.3,0,.2,.52));
 assert(!rollingSuccessorSafe(.25,0,-1,0,.2,.52));
 assert(!rollingSuccessorSafe(.25,0,0,1,.2,.52));
 assert(!rollingSuccessorSafe(0,0,1,0,.2,.52));
 assert(!rollingSuccessorSafe(.25,0,std::numeric_limits<double>::quiet_NaN(),0,.2,.52));
 assert(rollingSuccessorSafe(-.25,.001,-1,-.001,.2,.52));
 std::cout << "8 rolling handoff checks passed\n";
}
