#pragma once
#include <cmath>
// Kept independent of PX4 so the filtering can be tested without motors. Both run at the module's 250 Hz.

// The nose lift acts on the pitch rate. On the aircraft the front fans hunted at about 6 Hz against a gain of 0.02 per
// deg/s: the rate swung +/-15 deg/s around a nose that was barely moving, the fans hopped between 0 and 0.5, and the
// settle test (3 deg/s for half a second) never held (board logs 465, 467). One pole at 2 Hz takes a 6 Hz wobble down
// to a third and leaves the nose's own motion, well under 1 Hz, alone.
struct AtlasGroundFilter {
 static constexpr float kDt=0.004f, kHz=2.f;
 // Settled means the nose is staying at its target, not that the rate sensor is quiet. With the fans steady the
 // aircraft still shakes on its rear legs at about 5 Hz, +/-0.3 deg (+/-8 deg/s, run 20261031-183425); through this
 // filter that sat on the old 3 deg/s limit and the climb never started. A nose still travelling to its target
 // moves faster than this, and the angle test beside it (2 deg) holds the rest.
 static constexpr float kSettleRate=8.f;
 static float gain() { return kDt/(kDt+1.f/(2.f*3.14159265f*kHz)); }
 float q{0.f}, r{0.f};   // pitch and yaw rate, deg/s
 bool primed{false};
 void reset() { primed=false; q=0.f; r=0.f; }
 void update(float q_in, float r_in) {
  if (!std::isfinite(q_in) || !std::isfinite(r_in)) { return; }
  if (!primed) { q=q_in; r=r_in; primed=true; return; }
  const float a=gain();
  q+=a*(q_in-q); r+=a*(r_in-r);
 }
};

// How fast a front fan's command may change while the nose is lifted and held. With the loop on filtered rates the fans
// still pumped together between 0.29 and 0.59 at up to 2.6 per second, and every surge shook the nose, which the rate
// loop answered with the next surge (run 20261031-183425 and the attempt after it). The lift itself never asks for
// more than about 0.2 per second, so rising at 0.4 leaves it as it was and takes a surge up from a fraction of a
// second to one. Falling is five times quicker, and must be: when the nose breaks free the fans have to drop from
// about 0.8 to 0.45 at once, and limited to 0.4 per second both ways the nose ran on and lifted the aircraft off its
// feet (SITL, "abort: early liftoff").
// The nose lowering hunted the same way, at 5.6 Hz with the commands swinging 0 to 0.9 (board log 474), and gets the
// same treatment except that it may rise faster: below the balance point the fans have to catch a falling nose, and
// gravity's pull on it grows by the degree. Two per second still cuts a 6 Hz swing of 0.9 to under a fifth of it; one
// per second was the limit on every swing just after touchdown and made a bigger one (SITL, fans 0.13 to 0.8).
struct AtlasFrontSlew {
 static constexpr float kDt=0.004f, kUpPerSecond=0.4f, kDownPerSecond=2.f, kLowerUpPerSecond=2.f;
 static float step(float previous, float wanted, float up_per_second=kUpPerSecond) {
  if (!std::isfinite(wanted)) { return previous; }
  if (!std::isfinite(previous)) { return wanted; }
  const float up=up_per_second*kDt, down=kDownPerSecond*kDt;
  return wanted>previous+up ? previous+up : (wanted<previous-down ? previous-down : wanted);
 }
};

// On its rear feet the aircraft balances where its CG stands over them: 24 deg structural for the V4, from the ground
// model's CG about the rear-foot pivot. Above that it falls backwards, and the front fans, which can only lift the
// nose, cannot bring it back. On 31 Oct the V4 hovered at about 24 deg, landed on its rear feet at 23-25 deg, sat
// there for four seconds and went over onto its back. The rear motors sit behind the rear feet and lean forward, so
// on the feet their thrust pulls the nose down (about 0.19 N m per N on the V4). This is the share of each rear motor's
// thrust the nose lowering asks for: none more than kStart degrees below the balance point, then more the higher the
// nose and the faster it rises, never more than the cap (NLF_REAR_AST), which leaves the rear feet carrying the weight.
struct AtlasRearAssist {
 static constexpr float kDt=0.004f, kStart=10.f, kPerDeg=0.01f, kPerDegS=0.003f, kUpPerSecond=1.f, kDownPerSecond=0.5f;
 // CG ahead of (gx) and above (gz < 0) the rear-foot pivot, structural axes
 static float balance_deg(float gx, float gz) { return atan2f(gx, -gz)*57.2957795f; }
 static float wanted(float theta_deg, float q_deg_s, float balance_deg, float cap) {
  if (!std::isfinite(theta_deg) || !std::isfinite(balance_deg) || !(cap>0.f)) { return 0.f; }
  const float over=theta_deg-(balance_deg-kStart);
  if (!(over>0.f)) { return 0.f; }
  const float w=kPerDeg*over+kPerDegS*(std::isfinite(q_deg_s) ? fmaxf(q_deg_s,0.f) : 0.f);
  return w>cap ? cap : w;
 }
 // quick to come in (a nose past the balance point does not wait), slower to go
 static float step(float previous, float wanted) {
  const float up=kUpPerSecond*kDt, down=kDownPerSecond*kDt;
  return wanted>previous+up ? previous+up : (wanted<previous-down ? previous-down : wanted);
 }
};

// What the front fans have been pushing, averaged over about half a second, as thrust fractions. A ground abort ramps
// out from here rather than from the last command: while the fans hunted, the last command could be the top of a pulse
// (0.42 against an average near 0.2 in board log 467), and held for the two-second ramp it drove the nose from 11 deg
// over the balance point.
struct AtlasFrontAverage {
 static constexpr float kDt=0.004f, kTau=0.5f;
 float a{0.f}, b{0.f}, c{0.f};
 bool primed{false};
 void reset() { primed=false; a=b=c=0.f; }
 void update(float ta, float tb, float tc) {
  if (!std::isfinite(ta) || !std::isfinite(tb) || !std::isfinite(tc)) { return; }
  if (!primed) { a=ta; b=tb; c=tc; primed=true; return; }
  const float k=kDt/(kDt+kTau);
  a+=k*(ta-a); b+=k*(tb-b); c+=k*(tc-c);
 }
};
