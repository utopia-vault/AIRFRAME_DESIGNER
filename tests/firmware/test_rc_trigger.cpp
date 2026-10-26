#include <cassert>
#include <limits>
#include "../../firmware/atlas/src/atlas_nose_lift/RcTrigger.hpp"
int main(){
 float ch[18] = {0};
 // Channel 7 is the flight-mode switch: up fires the takeoff (7), down fires the landing (-7), the middle neither.
 ch[6] = 1.f;  assert(AtlasRcTrigger::active(7, ch, 18));  assert(!AtlasRcTrigger::active(-7, ch, 18));
 ch[6] = 0.f;  assert(!AtlasRcTrigger::active(7, ch, 18)); assert(!AtlasRcTrigger::active(-7, ch, 18));
 ch[6] = -1.f; assert(!AtlasRcTrigger::active(7, ch, 18)); assert(AtlasRcTrigger::active(-7, ch, 18));
 // Unmapped, out of range or not a number never fires.
 assert(!AtlasRcTrigger::active(0, ch, 18));
 assert(!AtlasRcTrigger::active(19, ch, 18)); assert(!AtlasRcTrigger::active(-19, ch, 18));
 assert(!AtlasRcTrigger::active(8, ch, 7));
 ch[6] = std::numeric_limits<float>::quiet_NaN(); assert(!AtlasRcTrigger::active(7, ch, 18)); assert(!AtlasRcTrigger::active(-7, ch, 18));

 // A switch already in place when first seen does nothing; moving it there fires once.
 AtlasRcTrigger t;
 assert(!t.edge(true));            // up at boot
 assert(!t.edge(true));
 assert(!t.edge(false));           // to the middle
 assert(t.edge(true));             // and back up: fires
 assert(!t.edge(true));            // held: not again
 assert(!t.edge(false)); assert(t.edge(true));
 t.forget(); assert(!t.edge(true)); // forgotten (parameter cleared): needs a fresh move
 assert(!t.edge(false)); assert(t.edge(true));
 return 0;
}
