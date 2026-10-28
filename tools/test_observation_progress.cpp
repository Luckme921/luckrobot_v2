#include "explore/observation_progress.hpp"
#include <cassert>
#include <iostream>
int main(){
 using explore::ObservationProgress;
 // Regression: region 22 shrank from 1.11 to 0.54; old point must stay suppressed.
 assert(ObservationProgress::repeat(0.02,0.09,1.11,0.54));
 assert(ObservationProgress::repeat(0.02,0.09,0.54,0.56));
 assert(!ObservationProgress::repeat(0.02,0.09,0.54,1.11));
 assert(!ObservationProgress::repeat(0.7,0.09,1.11,0.54));
 ObservationProgress p;
 assert(p.eligible(12,0,0,.69));p.reached(12);
 assert(p.eligible(12,0,0,.69));p.reached(12);
 assert(!p.eligible(12,0,0,.66));
 assert(!p.eligible(12,0,0,.55));
 // Separate branch remains eligible; no timer makes the stale branch reappear.
 assert(p.eligible(22,2,0,1.11));
 for(int i=0;i<100;++i)assert(!p.eligible(12,0,0,.55));
 assert(p.eligible(12,0,0,.50)); // meaningful new map coverage
 p.reached(12);p.reached(12);
 assert(p.eligible(12,1,0,.50)); // frontier advanced spatially
 std::cout<<"Observation repeat/shrink, bounded attempts, branch isolation and evidence reopening passed\n";
}
