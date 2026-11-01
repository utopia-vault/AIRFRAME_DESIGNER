#include <cassert>
#include <cmath>
#include <limits>
#include "../../firmware/atlas/src/atlas_nose_lift/GroundFilter.hpp"
int main(){
 const float pi=3.14159265f, dt=AtlasGroundFilter::kDt;
 // Board log 467: a nose all but still, with the front fans' own wobble on its rate, about 6 Hz and +/-15 deg/s.
 // The filtered rate has to come under the 3 deg/s settle limit; the raw one never stayed there for 0.5 s.
 AtlasGroundFilter f;
 float peak=0.f;
 for (int i=0;i<2500;i++) {                          // 10 s at 250 Hz
  const float t=i*dt, wobble=15.f*sinf(2.f*pi*6.f*t);
  f.update(wobble, -wobble);
  if (t>2.f) { peak=fmaxf(peak,fmaxf(fabsf(f.q),fabsf(f.r))); }
 }
 assert(peak<5.5f && peak>3.f);                      // one pole at 2 Hz: a third of it gets through
 assert(peak<AtlasGroundFilter::kSettleRate);         // which no longer counts as a moving nose
 // The shake on the rear legs with the fans steady (run 20261031-183425): about 5 Hz, +/-8 deg/s
 AtlasGroundFilter legs;
 float legs_peak=0.f;
 for (int i=0;i<2500;i++) {
  const float t=i*dt;
  legs.update(8.f*sinf(2.f*pi*5.f*t), 0.f);
  if (t>2.f) { legs_peak=fmaxf(legs_peak,fabsf(legs.q)); }
 }
 assert(legs_peak<AtlasGroundFilter::kSettleRate);
 // A nose travelling at the lift's own 5 deg/s is under the limit too, and the 2 deg angle test is what holds it back;
 // at 10 deg/s, faster than the lift ever asks for, it counts as moving.
 AtlasGroundFilter rising;
 for (int i=0;i<500;i++) { rising.update(10.f, 0.f); }
 assert(rising.q>AtlasGroundFilter::kSettleRate);
 // and the nose's own motion, well under 1 Hz, is left alone
 AtlasGroundFilter slow;
 float slow_peak=0.f;
 for (int i=0;i<5000;i++) {
  const float t=i*dt;
  slow.update(10.f*sinf(2.f*pi*0.2f*t), 0.f);
  if (t>5.f) { slow_peak=fmaxf(slow_peak,fabsf(slow.q)); }
 }
 assert(slow_peak>9.8f);
 // a steady rate comes straight through, a bad sample is skipped, and a reset starts from the next reading
 AtlasGroundFilter s;
 s.update(4.f,1.f); assert(s.q==4.f && s.r==1.f);
 for (int i=0;i<500;i++) { s.update(4.f,1.f); }
 assert(fabsf(s.q-4.f)<1e-4f);
 s.update(std::numeric_limits<float>::quiet_NaN(),0.f); assert(fabsf(s.q-4.f)<1e-4f);
 s.reset(); assert(!s.primed); s.update(-2.f,0.f); assert(s.q==-2.f);

 // The fans hunting between 0 and 0.5 command (thrust 0 and 0.25 on a quadratic fan), several times a second. The
 // abort used to start from whichever of the two the last sample happened to be; the average is what held the nose.
 AtlasFrontAverage avg;
 for (int i=0;i<2500;i++) {
  const float thrust=(static_cast<int>(i*dt*12.6f)%2) ? 0.25f : 0.f;   // 6.3 Hz square wave
  avg.update(thrust, thrust, thrust);
 }
 assert(avg.a>0.10f && avg.a<0.15f && avg.b==avg.a && avg.c==avg.a);
 // The surge seen on the aircraft, 0.29 to 0.59 in a few samples, now takes most of a second; a small change passes as is
 float c=0.29f; int steps=0;
 while (c<0.59f-1e-6f) { c=AtlasFrontSlew::step(c,0.59f); steps++; }
 assert(steps*AtlasFrontSlew::kDt>0.7f && steps*AtlasFrontSlew::kDt<0.8f);
 assert(fabsf(AtlasFrontSlew::step(0.4f,0.401f)-0.401f)<1e-6f);
 // and the drop when the nose breaks free, 0.8 to 0.45, takes under a fifth of a second
 c=0.8f; steps=0;
 while (c>0.45f+1e-6f) { c=AtlasFrontSlew::step(c,0.45f); steps++; }
 assert(steps*AtlasFrontSlew::kDt<0.2f);
 assert(AtlasFrontSlew::step(0.3f,std::numeric_limits<float>::quiet_NaN())==0.3f);
 // primed on its first sample, holds through a bad one, and forgets everything on reset
 AtlasFrontAverage one;
 one.update(0.2f,0.1f,0.f); assert(one.primed && one.a==0.2f && one.b==0.1f);
 one.update(std::numeric_limits<float>::infinity(),0.f,0.f); assert(one.a==0.2f);
 one.reset(); assert(!one.primed && one.a==0.f);
 return 0;
}
