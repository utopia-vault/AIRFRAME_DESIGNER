#pragma once
#include <cmath>
// The remote's takeoff and landing triggers (NLF_RC_CH, NLF_RC_LAND). A positive channel number fires when that
// channel goes high, a negative one when it goes low, so the takeoff and the landing can be the two ends of the
// flight-mode switch, standing in for PX4's Takeoff and Land slots. Kept independent of PX4 so it can be tested alone.
struct AtlasRcTrigger {
 // Is the mapped channel in its firing position? channels are PX4's calibrated rc_channels values, -1..1.
 static bool active(int mapped, const float *channels, int count) {
  const int c = mapped < 0 ? -mapped : mapped;
  if (c < 1 || c > count || !std::isfinite(channels[c - 1])) { return false; }
  return mapped > 0 ? channels[c - 1] > 0.5f : channels[c - 1] < -0.5f;
 }
 bool seen{false}, was{false};
 // True once, on the move into the firing position. A switch already there when first seen (at boot, or when the
 // parameter was just set) does nothing: it has to be moved there.
 bool edge(bool now) {
  const bool fire = seen && now && !was;
  seen = true; was = now;
  return fire;
 }
 void forget() { seen = false; }
};
