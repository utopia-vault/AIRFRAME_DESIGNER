// Experimental ATLAS ground-attitude sequencer. Free on SITL; on a flight controller only with NLF_HW_OK = 1.
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <drivers/drv_hrt.h>
#include <matrix/matrix/math.hpp>
#include <mathlib/mathlib.h>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/offboard_control_mode.h>
#include <uORB/topics/vehicle_attitude.h>

#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/trajectory_setpoint.h>
#include <px4_platform_common/atomic.h>   // NuttX has no std::atomic; px4::atomic works on SITL and boards
#include "Model.hpp"
#include "GroundFilter.hpp"
#include "PilotGate.hpp"
#include "PilotHandover.hpp"
#include "RcTrigger.hpp"
#include <uORB/topics/manual_control_setpoint.h>
#include "ManualGate.hpp"
#include "ManualAltitude.hpp"
#include <PositionControl.hpp>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/atlas_nose_lift_pilot.h>
#include <uORB/topics/actuator_motors.h>
#include <uORB/topics/atlas_nose_lift_floor.h>
#include <uORB/topics/vehicle_thrust_setpoint.h>
#include <uORB/topics/mavlink_log.h>
#include <uORB/topics/vehicle_command_ack.h>
#include <uORB/topics/rc_channels.h>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/estimator_aid_source1d.h>
#include <uORB/topics/register_ext_component_request.h>
#include <uORB/topics/register_ext_component_reply.h>
#include <uORB/topics/unregister_ext_component.h>
#include <uORB/topics/arming_check_request.h>
#include <uORB/topics/arming_check_reply.h>
#include <uORB/topics/setpoint_config.h>
#include <uORB/topics/vehicle_constraints.h>
#include <systemlib/mavlink_log.h>
using namespace time_literals;
// Separate stack for the sequencer: manual and automatic controller frames must
// not consume the small shared attitude/position-controller work queue stack.
static constexpr px4::wq_config_t atlas_sequence_queue{
 "wq:atlas_sequence", 6144, px4::wq_configurations::nav_and_controllers.relative_priority};

class AtlasNoseLift : public ModuleBase, public ModuleParams, public px4::ScheduledWorkItem {
public:
 static Descriptor desc;
 AtlasNoseLift(): ModuleParams(nullptr), ScheduledWorkItem(MODULE_NAME, atlas_sequence_queue) {}
 static int task_spawn(int, char**) {
  auto *p = new AtlasNoseLift();
  if (!p) { return PX4_ERROR; }
  desc.object.store(p); desc.task_id = task_id_is_work_queue;
  p->ScheduleOnInterval(4_ms); return PX4_OK;
 }
 static int custom_command(int argc, char **argv) {
  if (argc && !strcmp(argv[0], "takeoff") && desc.object.load()) {
   static_cast<AtlasNoseLift*>(desc.object.load())->_request.store(true); return PX4_OK;
  }
  if (argc && !strcmp(argv[0], "land") && desc.object.load()) {
   static_cast<AtlasNoseLift*>(desc.object.load())->_land_request.store(true); return PX4_OK;
  }
  return print_usage("start the module, then request takeoff or land");
 }
 static int print_usage(const char *reason = nullptr) {
  if (reason) { PX4_WARN("%s", reason); }
  PRINT_MODULE_USAGE_NAME("atlas_nose_lift", "controller");
  PRINT_MODULE_USAGE_COMMAND("start"); PRINT_MODULE_USAGE_COMMAND("takeoff"); PRINT_MODULE_USAGE_COMMAND("land");
  PRINT_MODULE_USAGE_DEFAULT_COMMANDS(); return 0;
 }
 int print_status() override {
  if (_manual_latched) { PX4_INFO("simulation manual phase=%d thrust=%.3f", int(_manual.state), double(_manual.thrust)); return 0; }
  PX4_INFO("phase=%d pitch=%.2f front9=%.3f", int(_phase), double(_pitch), double(_cmd9));
  PX4_INFO("modes: takeoff %d, land %d (nav states; -1 = PX4's own)%s", int(_own[TO].mode_id), int(_own[LAND].mode_id), _land_fallback ? ", landing handed to PX4" : "");
  return 0;
 }
private:
#include "ManualMode.hpp"
 enum Phase { Idle, Prime, Lift, Spool, Climb, Hover, Pilot, PilotGround, Retake, Descend, LowerNose, Shutdown, Aborting, Failed };   // Retake: taking a PX4 hover back into Offboard to land; Pilot/PilotGround: NLF_PILOT (PX4 flies / the module flies the ground with the pilot's throttle)
 Phase _phase{Idle};
 // Messages go to the console and, mirrored, to the ground station / designer app.
 orb_advert_t _mavlink_log_pub{nullptr};
#define NL_INFO(...) do { PX4_INFO(__VA_ARGS__); mavlink_log_info(&_mavlink_log_pub, __VA_ARGS__); } while (0)
#define NL_WARN(...) do { PX4_WARN(__VA_ARGS__); mavlink_log_warning(&_mavlink_log_pub, __VA_ARGS__); } while (0)
#define NL_ERR(...) do { PX4_ERR(__VA_ARGS__); mavlink_log_critical(&_mavlink_log_pub, __VA_ARGS__); } while (0)   // Aborting phase entered only to keep lockstep alive after an external disarm
 AtlasPilotGate _pilot_gate{};
 bool _pilot_mode{false}, _ready_announced{false};
 float _flight_alt{1.f}, _pilot_cap{0.f};
 bool _pilot_ground_abort{false};
 // PX4's own Land (an RC-loss failsafe, a ground station) ends with the rear feet on the ground, the nose up and PX4
 // cutting the thrust, which drops the nose. From the moment PX4's land detector sees that contact the module holds the
 // front fans through the allocator floor and lowers the nose as in its own landing; PX4 stays in Land throughout, and
 // the module lets go the moment PX4 leaves Land or disarms.
 bool _px4_land_takeover{false};
 float _pilot_abort_limit9{0.f}, _pilot_abort_limit10{0.f}, _pilot_abort_limitc{0.f};
 // Pilot handover (NLF_PILOT): after the nose lift PX4 goes to Stabilized or Altitude mode and the pilot's throttle lifts
 // the aircraft. Nothing in this path needs a position estimate. The front-fan floor (allocator overlay, max of PX4's
 // command and the floor) keeps the nose up while the feet may touch and under a throttle cut; a stalled descent at low
 // throttle is a landing and lowers the nose through the same floor.
 bool _pilot_handover{false}, _landing{false}, _handover_requested{false}, _takeover_request{false}, _was_high{false};
 uint8_t _pilot_nav{vehicle_status_s::NAVIGATION_STATE_STAB};
 float _lift{};   // commanded thrust over weight while the module flies the ground (1 = hover)
 hrt_abstime _liftoff_since{}, _takeover_since{}, _stick_lost_since{};
 uORB::Subscription _manual_sub{ORB_ID(manual_control_setpoint)};
 // The pilot's throttle as PX4's own controllers see it (RC receiver or a MAVLink joystick), [0,1]
 bool stick_throttle(hrt_abstime now, float &value) {
  manual_control_setpoint_s m{};
  if (!_manual_sub.copy(&m) || !m.valid || !m.timestamp || now<m.timestamp || now-m.timestamp>500_ms) { return false; }
  value=AtlasPilotHandover::throttle_from_setpoint(m.throttle);
  return value>=0.f;
 }
 const char *pilot_mode_name() const { return _pilot_nav==vehicle_status_s::NAVIGATION_STATE_ALTCTL ? "Altitude" : "Stabilized"; }
 float structural_pitch_deg(const vehicle_attitude_s &att) const {
  return math::degrees(matrix::Eulerf(matrix::Dcmf(matrix::Quatf(att.q))*matrix::Dcmf(matrix::Eulerf(0.f,math::radians(_hover_angle),0.f))).theta());
 }
 // The front fans are the model's (NLF_FA, NLF_FB: the yaw pair; NLF_FC: an optional third). _cmd9/_cmd10 are A and B,
 // _cmdc the third. Commands go through PX4's thrust curve: a thrust fraction t is (1 - f) sqrt(t) + f t.
 float to_cmd(float t) const { t=math::constrain(t,0.f,1.f); return (1.f-_ground.thr_fac)*sqrtf(t)+_ground.thr_fac*t; }
 float to_thrust(float c) const { c=math::constrain(c,0.f,1.f); return (1.f-_ground.thr_fac)*c*c+_ground.thr_fac*c; }
 void front_split(float frac, float delta) {
  _cmd9=to_cmd(frac*(_ground.w9+delta)); _cmd10=to_cmd(frac*(2.f-_ground.w9-delta));
  _cmdc=_ground.fc ? to_cmd(frac*_ground.wc) : 0.f;
 }
 void put_front(float *control, float a, float b, float c) const {
  control[_ground.fa-1]=a; control[_ground.fb-1]=b; if (_ground.fc) { control[_ground.fc-1]=c; }
 }
 bool is_front(int i) const { return i==_ground.fa-1 || i==_ground.fb-1 || (_ground.fc && i==_ground.fc-1); }
 void put_rear(float *control, float c) const {
  for (int i=0;i<math::min((int)_ground.count,12);i++) { if (!is_front(i)) { control[i]=math::constrain(c,0.f,1.f); } }
 }
 // After a boot PX4 ignores its range finder until the reading has moved 10 cm (a hard-coded stuck check) and fuses
 // a stand-in EKF2_MIN_RNG meanwhile. Lifting the nose on that let the height estimate drift on the stand (0.8 m in
 // HITL, board log 445): the land detector flipped to "in air", the handover started from a wrong height and the
 // climb overshot and sagged. So the sequence arms only once PX4 fuses a real reading. A design without a range
 // finder (EKF2_RNG_CTRL 0) has nothing to wait for.
 uORB::Subscription _rng_aid_sub{ORB_ID(estimator_aid_src_rng_hgt)};
 bool _rng_wait_announced{false}, _rng_latched{false};
 hrt_abstime _rng_ok_since{0};
 // In use = PX4 fused a real reading (not the stand-in) within the last half second. Its per-sample fused flag
 // flickers while it takes the range finder back, so the sequence asks for a second of it before it goes on.
 bool range_in_use(hrt_abstime now) {
  int32_t ctrl{0}; param_get(param_find("EKF2_RNG_CTRL"), &ctrl);
  if (ctrl == 0) { return true; }
  float min_rng{0.1f}; param_get(param_find("EKF2_MIN_RNG"), &min_rng);
  estimator_aid_source1d_s a{};
  if (!_rng_aid_sub.copy(&a) || !a.timestamp || now < a.timestamp || now - a.timestamp > 500_ms) { return false; }
  return a.time_last_fuse && now >= a.time_last_fuse && now - a.time_last_fuse < 500_ms
      && !a.innovation_rejected && a.observation > min_rng + 0.05f;
 }
 // Thrust input for PX4's land detector while the module drives the motors directly (not a landed flag): 0.16 per unit of
 // nose effort, which is 0.08 (t9 + t10) on the ten-motor layout. The aircraft is on its feet in every phase that
 // publishes it, so it stays below the detector's maybe-landed threshold (MPC_MANTHR_MIN + 0.01 outside climb-rate
 // control): PX4 believing it took off while the nose is coming up switches its estimator to in-air fusion, and with a
 // tilted range finder as the height reference that moves the height estimate by tenths of a metre on the spot.
 float front_thrust(float a, float b, float c) const {
  const float effort=0.16f*(to_thrust(a)+to_thrust(b)+(_ground.fc ? to_thrust(c) : 0.f))/(2.f+_ground.wc);
  return math::min(effort,0.8f*(_manthr_min.get()+0.01f));
 }
 void publish_floor_front(hrt_abstime now, const vehicle_attitude_s &att, float c9, float c10, float cc, float rear=NAN) {
  atlas_nose_lift_floor_s floor{}; floor.timestamp=now; floor.timestamp_sample=att.timestamp; floor.active=true;
  for (int i=0;i<12;i++) { floor.control[i]=NAN; }
  if (PX4_ISFINITE(rear)) { put_rear(floor.control,rear); }
  put_front(floor.control,math::constrain(c9,0.f,1.f),math::constrain(c10,0.f,1.f),math::constrain(cc,0.f,1.f));
  _floor_pub.publish(floor);
 }
 // Controlled nose lowering from the current front-fan commands (direct actuators in Offboard, the floor otherwise).
 void ground_lower(const vehicle_attitude_s &att, const vehicle_local_position_s &pos, hrt_abstime now, const char *why, bool capped) {
  _integral=0.f; _dwell=0; _contact_dwell=0; _phase_start=now;
  _pilot_ground_abort=capped; _pilot_abort_limit9=_cmd9; _pilot_abort_limit10=_cmd10; _pilot_abort_limitc=_cmdc;
  _phase=LowerNose; _touch_z=pos.z; _lower_target=structural_pitch_deg(att);
  NL_WARN("%s: lowering the nose", why);
 }
 uORB::Publication<atlas_nose_lift_pilot_s> _pilot_pub{ORB_ID(atlas_nose_lift_pilot)};
 bool pilot_throttle(hrt_abstime now, float &value) {
  rc_channels_s rc{};
  // rc_update sets timestamp from input_rc.timestamp_last_signal; timestamp_last_valid is unused.
  if (!_rc_sub.copy(&rc) || !AtlasPilotGate::rc_fresh(now,rc.timestamp,rc.signal_lost)) { return false; }
  const int channel=rc.function[rc_channels_s::FUNCTION_THROTTLE];
  if (channel<0 || channel>=rc.channel_count || channel>=18 || !PX4_ISFINITE(rc.channels[channel]) || fabsf(rc.channels[channel])>1.001f) { return false; }
  // rc_update publishes calibrated raw channels in [-1,1] (including throttle).
  value=math::constrain((rc.channels[channel]+1.f)*.5f,0.f,1.f);
  return true;
 }
 void pilot_abort(const vehicle_attitude_s &att, const vehicle_local_position_s &pos, hrt_abstime now) {
  clear_floor(); _integral=0.f; _dwell=0; _contact_dwell=0; _phase_start=now;
  if ((_phase==Prime || _phase==Lift) && _z-pos.z<.15f) {
   _pilot_ground_abort=true; _pilot_abort_limit9=_cmd9; _pilot_abort_limit10=_cmd10; _pilot_abort_limitc=_cmdc;
   _phase=LowerNose; _touch_z=pos.z;
   _lower_target=math::degrees(matrix::Eulerf(matrix::Dcmf(matrix::Quatf(att.q))*matrix::Dcmf(matrix::Eulerf(0.f,math::radians(_hover_angle),0.f))).theta());
   PX4_WARN("pilot abort: controlled nose lowering");
  } else {
   _phase=Descend; _target_z=pos.z; _x=pos.x; _y=pos.y;
   PX4_WARN("pilot abort: controlled landing");
  }
 }

 px4::atomic<bool> _request{false}, _land_request{false};
 static bool take(px4::atomic<bool> &flag) { bool expected = true; return flag.compare_exchange(&expected, false); }
 AtlasGroundModel _ground{};
 float _rest_pitch{}, _lower_target{}, _hover_angle{}, _landing_angle{};
 hrt_abstime _contact_dwell{};
 // What the range finder read standing on the legs at this takeoff (NAN when unknown): the landing trusts a quick
 // touchdown only within reach of it.
 float _park_range{NAN};
 float _touch_z{};
 hrt_abstime _start{}, _phase_start{}, _dwell{}, _last_command{};
 hrt_abstime _fade_start{};
 hrt_abstime _support_since{};   // Spool: when PX4's own allocation first matched the floor (must persist before fading)
 float _fade9{}, _fade10{}, _fadec{}, _abort9{}, _abort10{}, _abortc{};
 float _integral{}, _cmd9{}, _cmd10{}, _cmdc{};
 AtlasGroundFilter _lift_rates;     // structural pitch and yaw rate the lift loop and its settle test act on
 AtlasFrontAverage _front_avg;      // what the front fans pushed lately: a ground abort ramps out from this
 AtlasGroundFilter _lower_rates;    // pitch and yaw rate the lowering and its rear assist act on
 // The lowering's front commands are slew-limited (GroundFilter.hpp), but not from whatever the takeoff left in them:
 // after a flight its first step takes the loop's own value at once, as it always has (AtlasFrontSlew passes a wanted
 // value straight through from a non-finite one). Easing the front fans in from PX4's hover share instead keeps their
 // large nose-up moment about the rear feet, and past the balance point that tips the nose up (SITL, "abort: attitude
 // envelope").
 void restart_front_slew() { _cmd9=_cmd10=_cmdc=NAN; }
 float _rear_assist{};              // rear motors' thrust share pulling the nose down (GroundFilter.hpp)
 hrt_abstime _rear_assist_for{};    // the lowering it belongs to (its _phase_start)
 bool _rear_assist_said{false};
 hrt_abstime _diagnostic_time{};
 uint8_t _xy_reset{}, _z_reset{}, _heading_reset{};
 float _pitch{}, _yaw{}, _x{}, _y{}, _z{}, _target_z{};
 float _spool_z{}; hrt_abstime _rear_fade_start{};   // handover: height at spool start; when the rear floor started fading (airborne)
 bool _ground_ref{false};   // _z (ground height at the module's takeoff) is valid for this flight
 bool _was_armed{false};
 bool _released{false};
 uORB::Subscription _att_sub{ORB_ID(vehicle_attitude)}, _rates_sub{ORB_ID(vehicle_angular_velocity)},
  _pos_sub{ORB_ID(vehicle_local_position)}, _status_sub{ORB_ID(vehicle_status)}, _land_sub{ORB_ID(vehicle_land_detected)};
 uORB::Subscription _motors_sub{ORB_ID(atlas_nose_lift_feedback)};
 uORB::Publication<atlas_nose_lift_floor_s> _floor_pub{ORB_ID(atlas_nose_lift_floor)};
 uORB::Publication<vehicle_thrust_setpoint_s> _thrust_pub{ORB_ID(vehicle_thrust_setpoint)};
 uORB::Publication<actuator_motors_s> _motors_pub{ORB_ID(actuator_motors)};
 uORB::Publication<offboard_control_mode_s> _mode_pub{ORB_ID(offboard_control_mode)};

 uORB::Publication<trajectory_setpoint_s> _traj_pub{ORB_ID(trajectory_setpoint)};
 uORB::Publication<vehicle_command_s> _command_pub{ORB_ID(vehicle_command)};
 // External trigger, the same on SITL, HITL and the real aircraft: MAV_CMD_USER_1 (31010) with param1 = 1 (takeoff)
 // or 2 (land), sent by any ground station or the designer app. The shell commands remain for the console.
 static constexpr uint32_t CMD_ATLAS = 31010;
 // Remote-control triggers. NLF_RC_CH requests the takeoff and NLF_RC_LAND the landing when their channel moves into
 // its firing position: high for a positive channel number, low for a negative one, so the two can be the ends of the
 // flight-mode switch in place of PX4's Takeoff and Land slots (RcTrigger.hpp). A switch already there at boot does nothing.
 uORB::Subscription _rc_sub{ORB_ID(rc_channels)};
 // Kill switch: PX4 keeps the vehicle armed for 5 s after a kill and re-enables the motors if the switch comes back
 // within that time. Disarm at once instead, so releasing the switch can never restart anything.
 uORB::Subscription _armed_sub{ORB_ID(actuator_armed)};
 bool _kill_seen{false};
 void poll_kill(const vehicle_status_s &status) {
  actuator_armed_s aa{};
  if (!_armed_sub.update(&aa)) { return; }
  if (aa.kill && aa.armed && !_kill_seen) {
   command(vehicle_command_s::VEHICLE_CMD_COMPONENT_ARM_DISARM,0.f,21196.f,status);
   NL_WARN("kill switch: disarmed");
  }
  _kill_seen = aa.kill;
 }
 AtlasRcTrigger _trig_to{}, _trig_land{};
 void poll_rc_switch() {
  const int ct = _rc_ch.get(), cl = _rc_land.get();
  if (ct == 0) { _trig_to.forget(); }
  if (cl == 0) { _trig_land.forget(); }
  if (ct == 0 && cl == 0) { return; }
  rc_channels_s rc{};
  if (!_rc_sub.update(&rc)) { return; }
  if (rc.signal_lost) { return; }
  const int n = math::min((int)rc.channel_count, 18);
  if (ct != 0 && _trig_to.edge(AtlasRcTrigger::active(ct, rc.channels, n))) { _request.store(true); }
  if (cl != 0 && _trig_land.edge(AtlasRcTrigger::active(cl, rc.channels, n))) { _land_request.store(true); }
 }
 uORB::Subscription _cmd_sub{ORB_ID(vehicle_command)};
 uORB::Publication<vehicle_command_ack_s> _ack_pub{ORB_ID(vehicle_command_ack)};
 void poll_commands() {
  vehicle_command_s cmd{};
  while (_cmd_sub.update(&cmd)) {
   if (cmd.command != CMD_ATLAS) { continue; }
   const int what = (int)(cmd.param1 + 0.5f);
   if (what == 1) { _request.store(true); } else if (what == 2) { _land_request.store(true); }
   vehicle_command_ack_s ack{}; ack.timestamp=hrt_absolute_time(); ack.command=cmd.command;
   ack.result=(what == 1 || what == 2) ? vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED : vehicle_command_ack_s::VEHICLE_CMD_RESULT_DENIED;
   ack.target_system=cmd.source_system; ack.target_component=cmd.source_component; ack.from_external=false;
   _ack_pub.publish(ack);
  }
 }
 // PX4's Takeoff and Land, flown by this module (NLF_MODES = 1). The module registers two modes with the commander that
 // replace PX4's own (ModeManagement, the mechanism behind PX4's external modes), so whatever selects Takeoff or Land -
 // the flight-mode switch, a ground station, the takeoff command, a failsafe - selects the nose lift or the nose lowering
 // instead, and the switch slots keep their names. In its own mode the module tells the commander which setpoints it
 // sends (the motors directly while on the legs, a position on the way up and down), as offboard_control_mode does in
 // Offboard. The commander asks every registered mode three times a second whether it is alive and puts PX4's own back
 // within a second when one stops answering: that is how the module hands a landing it cannot fly to PX4's Land.
 struct OwnMode { const char *name; uint8_t replaces; uint64_t request_id; int8_t mode_id; int8_t check_id; uint16_t type; bool refused; };
 enum { TO=0, LAND=1 };
 OwnMode _own[2]{
  {"ATLAS Takeoff", vehicle_status_s::NAVIGATION_STATE_AUTO_TAKEOFF, 0x41544c4153544f01ULL, -1, -1, 0, false},
  {"ATLAS Land", vehicle_status_s::NAVIGATION_STATE_AUTO_LAND, 0x41544c41534c4401ULL, -1, -1, 0, false}};
 hrt_abstime _register_sent{}, _setpoint_sent{};
 bool _land_fallback{false};   // the module gave a landing up and PX4's own Land flies it; cleared on disarm
 uint8_t _last_nav{0xff};
 uORB::Publication<register_ext_component_request_s> _register_pub{ORB_ID(register_ext_component_request)};
 uORB::Publication<unregister_ext_component_s> _unregister_pub{ORB_ID(unregister_ext_component)};
 uORB::Publication<arming_check_reply_s> _check_reply_pub{ORB_ID(arming_check_reply)};
 uORB::Publication<setpoint_config_s> _setpoint_config_pub{ORB_ID(setpoint_config)};
 uORB::Publication<vehicle_constraints_s> _constraints_pub{ORB_ID(vehicle_constraints)};
 uORB::Subscription _register_reply_sub{ORB_ID(register_ext_component_reply)};
 uORB::Subscription _check_request_sub{ORB_ID(arming_check_request)};
 bool in_mode(const vehicle_status_s &s, int which) const { return _own[which].mode_id>=0 && s.nav_state==(uint8_t)_own[which].mode_id; }
 bool modes_ready() const { return _own[TO].mode_id>=0 && _own[LAND].mode_id>=0; }
 // The module flies the vehicle: in Offboard (asked for without the modes, and by the pilot handover) or in its own modes.
 bool owned(const vehicle_status_s &s) const { return s.nav_state==vehicle_status_s::NAVIGATION_STATE_OFFBOARD || in_mode(s,TO) || in_mode(s,LAND); }
 // The mode a takeoff (land = false) or a landing is flown in: the module's own, or Offboard without them.
 bool in_flying_mode(const vehicle_status_s &s, bool land) const {
  return modes_ready() ? in_mode(s,land ? LAND : TO) : s.nav_state==vehicle_status_s::NAVIGATION_STATE_OFFBOARD;
 }
 // Asking for PX4's Takeoff (AUTO 2) or Land (AUTO 6) is asking for the module's own: the commander swaps them.
 void request_flying_mode(const vehicle_status_s &status, bool land) {
  if (modes_ready()) { command(vehicle_command_s::VEHICLE_CMD_DO_SET_MODE,1.f,4.f,status,land ? 6.f : 2.f); }
  else { command(vehicle_command_s::VEHICLE_CMD_DO_SET_MODE,1.f,6.f,status); }
 }
 static int32_t param_int(const char *name) { int32_t v=0; param_get(param_find(name),&v); return v; }
 void unregister_mode(OwnMode &m) {
  unregister_ext_component_s u{}; u.timestamp=hrt_absolute_time();
  strncpy(u.name,m.name,sizeof(u.name)-1); u.arming_check_id=m.check_id; u.mode_id=m.mode_id;
  _unregister_pub.publish(u); m.mode_id=-1; m.check_id=-1; m.type=0;
 }
 void manage_modes(bool armed, hrt_abstime now) {
  register_ext_component_reply_s reply{};
  while (_register_reply_sub.update(&reply)) {
   for (auto &m : _own) {
    if (reply.request_id!=m.request_id || m.mode_id>=0) { continue; }
    if (reply.success && reply.mode_id>=0) {
     m.mode_id=reply.mode_id; m.check_id=reply.arming_check_id; m.type=0;
     // The number is what a ground station sees in the heartbeat (Auto, sub-mode 10 + n): the app names the mode from
     // this line, which has to fit in one STATUSTEXT (50 characters).
     NL_INFO("%s (external mode %d) replaces %s", m.name, int(m.mode_id)-vehicle_status_s::NAVIGATION_STATE_EXTERNAL1+1,
             m.replaces==vehicle_status_s::NAVIGATION_STATE_AUTO_TAKEOFF ? "Takeoff" : "Land");
    } else if (!m.refused) { m.refused=true; NL_ERR("PX4 refused the %s mode: its own stays", m.name); }
   }
  }
  arming_check_request_s request{};
  if (_check_request_sub.update(&request)) {
   for (int i=0;i<2;i++) {
    // A landing given up is PX4's: not answering makes the commander fly its own Land instead of this mode.
    if (_own[i].check_id<0 || (i==LAND && _land_fallback)) { continue; }
    arming_check_reply_s r{}; r.timestamp=now; r.request_id=request.request_id; r.registration_id=_own[i].check_id;
    r.health_component_index=arming_check_reply_s::HEALTH_COMPONENT_INDEX_NONE; r.can_arm_and_run=true;
    // A position is needed to fly, not to sit on the legs: once the nose is down PX4 can lose it (the H-flow looks at
    // the ground from a few centimetres), and a mode that demanded it then would be failsafed out of its own shutdown.
    const bool flying=!direct_phase();
    r.mode_req_angular_velocity=true; r.mode_req_attitude=true; r.mode_req_local_alt=flying; r.mode_req_local_position=flying;
    _check_reply_pub.publish(r);
   }
  }
  if (armed || now-_register_sent<1_s) { return; }   // the commander takes registrations on the ground only
  _register_sent=now;
  if (_land_fallback) {
   // Landed after giving a landing up: register the landing mode afresh, so it does not stay marked unresponsive.
   _land_fallback=false;
   if (_own[LAND].mode_id>=0) { unregister_mode(_own[LAND]); return; }
  }
  bool want=param_int("NLF_ENABLE")!=0 && param_int("NLF_MODES")!=0;
#if !defined(CONFIG_ARCH_BOARD_PX4_SITL)
  want=want && param_int("NLF_HW_OK")==1;
#endif
  for (auto &m : _own) {
   if (want && m.mode_id<0 && !m.refused) {
    register_ext_component_request_s r{}; r.timestamp=now; r.request_id=m.request_id;
    strncpy(r.name,m.name,sizeof(r.name)-1);
    r.px4_ros2_api_version=register_ext_component_request_s::LATEST_PX4_ROS2_API_VERSION;
    r.register_arming_check=true; r.register_mode=true;
    r.enable_replace_internal_mode=true; r.replace_internal_mode=m.replaces;
    _register_pub.publish(r);
   } else if (!want && m.mode_id>=0) {
    NL_INFO("%s removed: PX4's own is back", m.name); unregister_mode(m);
   }
   if (!want) { m.refused=false; }
  }
 }
 // A position setpoint in the module's own mode. PX4's flight tasks, which normally tell the position controller that a
 // setpoint above the vehicle is a takeoff, do not run in a registered mode, and without that the controller holds the
 // motors at idle on the ground: say it here exactly as mc_pos_control says it for itself in Offboard.
 void publish_trajectory(trajectory_setpoint_s &sp, const vehicle_status_s &status, const vehicle_local_position_s &pos, bool armed) {
  _traj_pub.publish(sp);
  if (!in_mode(status,TO) && !in_mode(status,LAND)) { return; }
  vehicle_constraints_s c{}; c.timestamp=sp.timestamp;
  c.speed_up=param_float("MPC_Z_VEL_MAX_UP"); c.speed_down=param_float("MPC_Z_VEL_MAX_DN");
  c.want_takeoff=armed && ((PX4_ISFINITE(sp.position[2]) && sp.position[2]<pos.z) || (PX4_ISFINITE(sp.velocity[2]) && sp.velocity[2]<0.f));
  _constraints_pub.publish(c);
 }
 static float param_float(const char *name) { float v=NAN; param_get(param_find(name),&v); return v; }
 // Which setpoints the module sends in its own mode: the motors directly while the aircraft is on its legs, a position
 // otherwise. The commander resets a mode to position setpoints whenever it is entered, so this is said again on entry.
 bool direct_phase() const { return _phase==Prime || _phase==Lift || _phase==LowerNose || _phase==Aborting || _phase==Shutdown || _phase==PilotGround; }
 void configure_setpoints(const vehicle_status_s &status, hrt_abstime now) {
  const bool direct=direct_phase();
  for (auto &m : _own) {
   if (m.mode_id<0 || status.nav_state!=(uint8_t)m.mode_id) { m.type=0; continue; }
   const uint16_t type=direct ? setpoint_config_s::TYPE_DIRECT_ACTUATORS : setpoint_config_s::TYPE_TRAJECTORY;
   if (type!=m.type || now-_setpoint_sent>500_ms) {
    setpoint_config_s c{}; c.timestamp=now; c.type=type; c.source_id=m.mode_id; c.should_apply=true;
    _setpoint_config_pub.publish(c); m.type=type; _setpoint_sent=now;
   }
  }
 }
 DEFINE_PARAMETERS(
  (ParamInt<px4::params::NLF_ENABLE>) _enable,
  (ParamInt<px4::params::NLF_MANUAL>) _manual_enable,
  (ParamInt<px4::params::NLF_RC_THR>) _rc_throttle,
  (ParamInt<px4::params::NLF_PILOT>) _pilot,
  (ParamFloat<px4::params::NLF_LMAX>) _lmax,
  (ParamFloat<px4::params::NLF_UF>) _uf,
  (ParamFloat<px4::params::NLF_U1>) _u1, (ParamFloat<px4::params::NLF_U2>) _u2, (ParamFloat<px4::params::NLF_U3>) _u3, (ParamFloat<px4::params::NLF_U4>) _u4,
  (ParamFloat<px4::params::NLF_U5>) _u5, (ParamFloat<px4::params::NLF_U6>) _u6, (ParamFloat<px4::params::NLF_U7>) _u7, (ParamFloat<px4::params::NLF_U8>) _u8,
  (ParamFloat<px4::params::COM_DISARM_PRFLT>) _prflt_delay,
  (ParamFloat<px4::params::NLF_RATE>) _rate,
  (ParamFloat<px4::params::NLF_TARGET>) _lift_target,
 (ParamInt<px4::params::NLF_HW_OK>) _hw_ok,
 (ParamFloat<px4::params::NLF_SPOOL_CMD>) _spool_cmd,
 (ParamFloat<px4::params::NLF_SPOOL_T>) _spool_t,
 (ParamInt<px4::params::NLF_RC_CH>) _rc_ch,
 (ParamInt<px4::params::NLF_RC_LAND>) _rc_land,
 (ParamInt<px4::params::NLF_MODES>) _modes_param,
  (ParamFloat<px4::params::SENS_BOARD_Y_OFF>) _hover_param,
  (ParamFloat<px4::params::NLF_LAND_ANG>) _land_angle_param,

  (ParamFloat<px4::params::NLF_RAMP>) _ramp,
  (ParamFloat<px4::params::NLF_DWELL>) _hold,
  (ParamFloat<px4::params::NLF_ALT>) _alt,
  (ParamFloat<px4::params::NLF_M_HOV>) _manual_hover,
  (ParamFloat<px4::params::NLF_TIMEOUT>) _timeout,
  (ParamFloat<px4::params::NLF_LAND_V>) _land_speed,
  (ParamFloat<px4::params::NLF_DOWN_R>) _down_rate,
  (ParamFloat<px4::params::NLF_REAR_AST>) _rear_assist_cap,
  (ParamFloat<px4::params::COM_DISARM_LAND>) _disarm_delay,
  (ParamFloat<px4::params::MPC_MANTHR_MIN>) _manthr_min
 )
 void command(uint32_t cmd, float p1, float p2, const vehicle_status_s &status, float p3=0.f) {
  vehicle_command_s c{}; c.timestamp=hrt_absolute_time(); c.command=cmd; c.param1=p1; c.param2=p2; c.param3=p3;
  c.target_system=status.system_id; c.target_component=status.component_id;
  c.source_system=status.system_id; c.source_component=status.component_id; c.from_external=false;
  _command_pub.publish(c);
 }
 void clear_floor() {
  atlas_nose_lift_floor_s floor{}; floor.timestamp=hrt_absolute_time(); floor.active=false;
  _floor_pub.publish(floor);
 }
 void fail(const char *reason, const vehicle_status_s &status, bool airborne) {
  NL_ERR("abort: %s", reason); clear_floor();
  if (_px4_land_takeover) { _px4_land_takeover=false; _phase=Idle; NL_WARN("the landing is PX4's again"); return; }
  // An estimator fault during a bounded ground abort must not hand low-power
  // direct control to a full-thrust position landing controller.
  if (_pilot_ground_abort && _phase==LowerNose) { airborne=false; }
  if (status.arming_state != vehicle_status_s::ARMING_STATE_ARMED) { _phase=Failed; return; }
  if (_pilot_handover && !owned(status)) {
   // PX4 is flying for the pilot: never request an automatic landing (no position); the floor is gone, the pilot has the throttle
   NL_WARN("pilot has the aircraft"); _phase=Failed; return;
  }
  if (_pilot_handover && (_phase==Pilot || (_phase==PilotGround && (_handover_requested || _landing)))) {
   // off the ground under the module's own motor commands: the only safe place for the aircraft is the pilot's mode
   command(vehicle_command_s::VEHICLE_CMD_DO_SET_MODE,1.f,_pilot_nav==vehicle_status_s::NAVIGATION_STATE_ALTCTL ? 2.f : 7.f,status);
   NL_WARN("handing PX4 %s the aircraft", pilot_mode_name()); _phase=Failed; return;
  }
  if (!airborne && owned(status)) {
   const bool lifting=(_phase==Lift || _phase==Spool) && _front_avg.primed;
   // (a lowering that has not run a step yet holds no front command: restart_front_slew)
   _phase=Aborting; _phase_start=hrt_absolute_time();
   _abort9=PX4_ISFINITE(_cmd9) ? _cmd9 : 0.f; _abort10=PX4_ISFINITE(_cmd10) ? _cmd10 : 0.f; _abortc=PX4_ISFINITE(_cmdc) ? _cmdc : 0.f;
   if (lifting) {
    // Ramp out from what held the nose, never from the last command: that can be the top of a pulse (GroundFilter.hpp),
    // or the bottom of one, which would drop the nose onto its feet with nothing under it.
    _abort9=to_cmd(_front_avg.a); _abort10=to_cmd(_front_avg.b); _abortc=_ground.fc ? to_cmd(_front_avg.c) : 0.f;
   }
   return;
  }
  _phase=Failed;
  if (modes_ready()) { _land_fallback=true; NL_WARN("PX4's own Land takes the landing"); }
  command(vehicle_command_s::VEHICLE_CMD_NAV_LAND,0.f,0.f,status);
 }
 void Run() override {
  if (!_manual_selected) { updateParams(); _manual_selected=true; _manual_latched=_manual_enable.get()!=0; }
  if (_manual_latched) {
#if defined(CONFIG_ARCH_BOARD_PX4_SITL)
   run_manual();
#else
   vehicle_status_s sim_status{}; _status_sub.copy(&sim_status);
   int32_t hitl=0; param_get(param_find("SYS_HITL"), &hitl);
   const auto sim_now=hrt_absolute_time();
   const bool fresh=sim_status.timestamp && sim_now>=sim_status.timestamp && sim_now-sim_status.timestamp<2_s;
   if (AtlasManualGate::simulation_allowed(false,hitl,sim_status.hil_state==vehicle_status_s::HIL_STATE_ON,fresh)) {
    run_manual();
   } else {
    // Never enter this prototype on real sensors, even if its parameter was retained after a reboot.
    clear_floor(); _manual.state=AtlasManualGate::Failed;
    take(_request); take(_land_request);
    if (should_exit()) { ScheduleClear(); exit_and_cleanup(desc); }
   }
#endif
   return;
  }
  run_automatic();
 }
 // Keep this frame out of Run, including optimized board builds.
 __attribute__((noinline)) void run_automatic() {
  poll_commands();
  poll_rc_switch();
  vehicle_status_s status{}; _status_sub.copy(&status);
  poll_kill(status);
  vehicle_attitude_s att{}; _att_sub.copy(&att);
  vehicle_angular_velocity_s rates{}; _rates_sub.copy(&rates);
  vehicle_local_position_s pos{}; _pos_sub.copy(&pos);
  vehicle_land_detected_s land{}; _land_sub.copy(&land);
  const auto now=hrt_absolute_time();
  const bool armed=status.arming_state==vehicle_status_s::ARMING_STATE_ARMED;
  manage_modes(armed, now);
  if (status.nav_state!=_last_nav) {
   // Selecting Takeoff or Land (switch, ground station, failsafe) is the request; the module's own requests for its
   // mode arrive here too, in a phase that is already under way. The line names the mode for a ground station that
   // missed the registration (the heartbeat carries only its number).
   for (const auto &m : _own) {
    if (m.mode_id>=0 && status.nav_state==(uint8_t)m.mode_id) { NL_INFO("%s (external mode %d)", m.name, int(m.mode_id)-vehicle_status_s::NAVIGATION_STATE_EXTERNAL1+1); }
   }
   if (in_mode(status,TO) && (_phase==Idle || _phase==Failed)) { _request.store(true); }
   if (in_mode(status,LAND) && _phase!=Retake && !_land_fallback) { _land_request.store(true); }
   _last_nav=status.nav_state;
  }
  configure_setpoints(status, now);
  atlas_nose_lift_pilot_s pilot{}; pilot.timestamp=now;
  // Keep throttle ownership through Shutdown/Aborting until disarmed. Otherwise
  // a high stick becomes a Position-mode climb request during the motor ramp-down.
  pilot.active=AtlasPilotGate::owns_throttle(_pilot_mode, _phase!=Idle, armed,
      owned(status) || status.nav_state==vehicle_status_s::NAVIGATION_STATE_AUTO_LAND)
      || (_pilot_handover && armed && (_phase==Prime || _phase==Lift || _phase==PilotGround));   // throttle motion must not trip the stick override
  _pilot_pub.publish(pilot);
  float throttle=0.f;
  const bool rc_valid=pilot_throttle(now, throttle);
  float stick=0.f;
  const bool stick_valid=stick_throttle(now, stick);
  bool second_click=false;
  if (should_exit()) {
   if (_phase>=Prime && _phase<=LowerNose) { fail("module stopped",status,_phase!=Prime && _phase!=Lift); }
   if (_phase!=Aborting && _phase!=Shutdown) { clear_floor(); ScheduleClear(); exit_and_cleanup(desc); return; }
  }
  if (_phase==Shutdown && (_pilot_handover || _px4_land_takeover) && !owned(status)) {
   // Pilot handover or PX4's Land: the nose rests on its feet, PX4 is still in its own mode. Ramp the floor out and disarm.
   if (!armed) { clear_floor(); NL_INFO("landing complete: all motors off and disarmed"); _phase=Idle; _px4_land_takeover=false; return; }
   const float scale=math::constrain(1.f-(now-_phase_start)*1e-6f/2.f,0.f,1.f);
   publish_floor_front(now,att,_abort9*scale,_abort10*scale,_abortc*scale);
   if (now-_phase_start>2500_ms && now-_last_command>1_s) {
    // PX4 declines a plain disarm until its land detector agrees; the nose has rested for seconds by then, so force it after 8 s
    command(vehicle_command_s::VEHICLE_CMD_COMPONENT_ARM_DISARM,0.f,now-_phase_start>8_s ? 21196.f : 0.f,status); _last_command=now;
   }
   return;
  }
  if (_phase==Aborting || _phase==Shutdown) {
   if (!owned(status)) {
    if (_phase==Shutdown) { if (!armed) { NL_INFO("landing complete: all motors off and disarmed"); } else { NL_WARN("landing interrupted by mode change"); } _phase=Idle; }
    else { _phase=_released ? Idle : Failed; if (_released) { NL_INFO("control handed back to PX4"); } _released=false; }
    return;
   }
   // Keep zero actuator samples flowing until the commander leaves direct control (into Manual).
   // Otherwise lockstep SITL can stop advancing after normal ground disarm.
   if (!armed && now-_last_command>1_s) {
    command(vehicle_command_s::VEHICLE_CMD_DO_SET_MODE,1.f,1.f,status); _last_command=now;   // Manual: safe to re-arm on the ground
   }
   const float scale=math::constrain(1.f-(now-_phase_start)*1e-6f/2.f,0.f,1.f);
   offboard_control_mode_s mode{}; mode.timestamp=now; mode.direct_actuator=true; _mode_pub.publish(mode);
   actuator_motors_s motors{}; motors.timestamp=now; motors.timestamp_sample=att.timestamp;
   for(int i=0;i<12;i++) { motors.control[i]=NAN; }
   put_front(motors.control,_abort9*scale,_abort10*scale,_abortc*scale); _motors_pub.publish(motors);
   vehicle_thrust_setpoint_s thrust{}; thrust.timestamp=now; thrust.timestamp_sample=att.timestamp;
   thrust.xyz[2]=-front_thrust(_abort9*scale,_abort10*scale,_abortc*scale);
   _thrust_pub.publish(thrust);
   if (armed && now-_phase_start>2500_ms && now-_last_command>1_s) {
    command(vehicle_command_s::VEHICLE_CMD_COMPONENT_ARM_DISARM,0.f,0.f,status); _last_command=now;
   }
   return;
  }
  const matrix::Eulerf e(matrix::Quatf(att.q)); _pitch=e.theta();
  if (take(_request)) {
   updateParams();
   if ((_pilot_mode || _pilot_handover) && _phase==Lift) {
    second_click=true;
    if (_pilot_handover && !_ready_announced) { NL_INFO("nose not settled yet: press Takeoff again after 'nose ready'"); }
   } else if (_phase!=Idle && _phase!=Failed) { NL_WARN("sequence already active"); }
   else {
   const bool want_pilot=_pilot.get()!=0;
   if (want_pilot) {
    if (!stick_valid || stick>AtlasPilotHandover::kHandoverThrottle) { NL_ERR("pilot handover needs the remote with the throttle at minimum"); return; }
    if (_rc_throttle.get()) { NL_WARN("NLF_RC_THR ignored: the pilot handover lifts the nose by itself"); }
    int32_t airmode=0; param_get(param_find("MC_AIRMODE"), &airmode);
    if (airmode!=0) { NL_ERR("pilot handover needs MC_AIRMODE 0: airmode trades thrust for pitch on the legs (Update PX4)"); return; }
    const float pattern[9]={_u1.get(),_u2.get(),_u3.get(),_u4.get(),_u5.get(),_u6.get(),_u7.get(),_u8.get(),_uf.get()};
    for (float v : pattern) { if (!PX4_ISFINITE(v) || v<0.02f || v>0.95f) { NL_ERR("pilot handover needs the hover pattern NLF_U1..8, NLF_UF (Update PX4)"); return; } }
    if (!PX4_ISFINITE(_lmax.get()) || _lmax.get()<1.05f || _lmax.get()>2.f) { NL_ERR("invalid NLF_LMAX"); return; }
   } else if (_rc_throttle.get() && !AtlasPilotGate::can_start(rc_valid,throttle)) { NL_ERR("staged takeoff needs fresh physical RC and throttle at minimum"); return; }
#if !defined(CONFIG_ARCH_BOARD_PX4_SITL)
   // Hardware interlock: on a flight controller the experimental sequence runs only after NLF_HW_OK was set by hand.
   if (_hw_ok.get() != 1) { NL_ERR("experimental module on hardware: set NLF_HW_OK = 1 to allow it"); return; }
#endif
   _hover_angle=_hover_param.get();
   if (!atlas_model_matches(_hover_angle, _ground)) { NL_ERR("ground model configuration invalid; Update PX4"); return; }
   if (want_pilot && !_ground.ten_motor_pair()) { NL_ERR("pilot handover needs the ten-motor layout (its hover pattern is eight rear motors)"); return; }
   if (!_enable.get() || armed || !land.landed || (!want_pilot && !pos.xy_valid) || !pos.z_valid || now-att.timestamp>200_ms
       || !PX4_ISFINITE(_pitch) || !PX4_ISFINITE(e.psi()) || !PX4_ISFINITE(pos.z)) {
    NL_ERR(want_pilot ? "requires enabled, disarmed, landed, fresh valid attitude and height" : "requires enabled, disarmed, landed, fresh valid attitude and position"); return;
   }
   if (want_pilot && (!PX4_ISFINITE(_down_rate.get()) || _down_rate.get()<1.f || _down_rate.get()>5.f || _disarm_delay.get()<90.f ||
       (_prflt_delay.get()>=0.f && _prflt_delay.get()<90.f))) {
    NL_ERR("pilot handover needs a valid nose lowering rate and COM_DISARM_LAND, COM_DISARM_PRFLT of 90 s or more (Update PX4)"); return;
   }
   if (!PX4_ISFINITE(_land_angle_param.get()) || !PX4_ISFINITE(_hover_angle) || !PX4_ISFINITE(_lift_target.get()) ||
       !PX4_ISFINITE(_rate.get()) || _rate.get()<1.f || _rate.get()>15.f ||
       !PX4_ISFINITE(_ramp.get()) || _ramp.get()<1.f || _ramp.get()>10.f ||
       !PX4_ISFINITE(_hold.get()) || _hold.get()<0.5f || _hold.get()>5.f ||
       !PX4_ISFINITE(_alt.get()) || _alt.get()<0.5f || _alt.get()>5.f ||
       !PX4_ISFINITE(_timeout.get()) || _timeout.get()<10.f || _timeout.get()>60.f) {
    NL_ERR("invalid sequence parameters"); return;
   }
   if (_rc_throttle.get() && (!PX4_ISFINITE(_down_rate.get()) || _down_rate.get()<1.f || _down_rate.get()>5.f ||
       !PX4_ISFINITE(_land_speed.get()) || _land_speed.get()<.05f || _land_speed.get()>.3f || _disarm_delay.get()<90.f)) {
    NL_ERR("staged mode needs valid landing rates and COM_DISARM_LAND >= 90 s"); return;
   }
   _pilot_handover=want_pilot; _pilot_nav=_pilot.get()==2 ? vehicle_status_s::NAVIGATION_STATE_ALTCTL : vehicle_status_s::NAVIGATION_STATE_STAB;
   _px4_land_takeover=false; _landing=false; _handover_requested=false; _takeover_request=false; _was_high=false; _lift=0.f; _liftoff_since=0; _takeover_since=0; _stick_lost_since=0; _contact_dwell=0;
   _pilot_mode=!want_pilot && _rc_throttle.get()!=0; _pilot_gate.reset(); _pilot_cap=0.f; _pilot_ground_abort=false; _ready_announced=false; _flight_alt=_pilot_mode ? 1.f : _alt.get();
   _phase=Prime; _start=now; _phase_start=now; _dwell=0; _fade_start=0; _integral=0.f; _cmd9=0.f; _cmd10=0.f; _cmdc=0.f; _last_command=0; _was_armed=false; _rng_wait_announced=false; _rng_latched=false; _rng_ok_since=0;
   _rest_pitch=math::degrees(matrix::Eulerf(matrix::Dcmf(matrix::Quatf(att.q))*matrix::Dcmf(matrix::Eulerf(0.f,math::radians(_hover_angle),0.f))).theta());
   _landing_angle=_land_angle_param.get();
   if (_landing_angle > _rest_pitch+3.f) {
    NL_ERR("landed pitch %.1f is above measured ground posture %.1f; adjust landed pitch to the legs", double(_landing_angle), double(_rest_pitch));
    _phase=Idle; return;
   }
   _xy_reset=pos.xy_reset_counter; _z_reset=pos.z_reset_counter; _heading_reset=pos.heading_reset_counter;
   _yaw=e.psi(); _x=pos.x; _y=pos.y; _z=pos.z; _target_z=_z; _ground_ref=true;
   _park_range=(pos.dist_bottom_valid && PX4_ISFINITE(pos.dist_bottom)) ? pos.dist_bottom : NAN;
   NL_INFO("priming nose lift at pitch %.1f",double(math::degrees(_pitch)));
   if (_pilot_mode) { NL_INFO("staged: raise RC throttle slowly; second takeoff press required"); }
   if (_pilot_handover) { NL_INFO("pilot handover (%s): keep the throttle at minimum until 'nose ready'", pilot_mode_name()); }
   }
  }
  if (take(_land_request)) {
   updateParams();
   if (_pilot_handover && _phase!=Idle && _phase!=Failed) {
    if (!armed) { _phase=Idle; clear_floor(); }
    else if (_phase==Prime || _phase==Lift || (_phase==PilotGround && !_landing && !_handover_requested)) { ground_lower(att,pos,now,"pilot requested abort",true); }
    else if (_phase==Pilot) { _takeover_request=true; }
    else if (_phase==PilotGround && _handover_requested) { NL_INFO("handing over to PX4; press Land again once it flies"); }
    else { NL_WARN("landing already in progress"); }
   } else if (_pilot_mode && (_phase==Prime || _phase==Lift || _phase==Spool || _phase==Climb)) {
    if (armed) { pilot_abort(att,pos,now); NL_INFO("pilot requested abort/landing"); }
    else { _phase=Idle; clear_floor(); }
   } else if (_land_fallback) {
    NL_WARN("the landing is PX4's");
   } else if (_phase==Prime) {
    _phase=Idle; clear_floor(); NL_INFO("takeoff cancelled");
    if (armed) { command(vehicle_command_s::VEHICLE_CMD_COMPONENT_ARM_DISARM,0.f,0.f,status); }
   } else if (_phase==Lift) {
    ground_lower(att,pos,now,"landing during the nose lift",false);
   } else if (_phase==Spool || _phase==Climb) {
    _phase=Descend; _phase_start=now; _contact_dwell=0; _target_z=pos.z; _x=pos.x; _y=pos.y; clear_floor();
    NL_INFO("landing: descending onto rear legs");
   } else {
   // from the module's own hover, or from any PX4 hover (e.g. Position mode after the pilot took over); the landing
   // finds the ground by itself, so the takeoff spot and its height do not matter
   const bool own_hover = _phase==Hover;
   const bool other_hover = (_phase==Idle || _phase==Failed) && armed && !land.landed;
   if (!(own_hover || other_hover) || !armed || !_enable.get() || !atlas_model_matches(_hover_angle, _ground)) {
    NL_WARN("landing needs an armed hover and a matching model");
   } else if (!PX4_ISFINITE(_land_speed.get()) || _land_speed.get()<0.05f || _land_speed.get()>0.3f ||
              !PX4_ISFINITE(_down_rate.get()) || _down_rate.get()<1.f || _down_rate.get()>5.f) {
    NL_ERR("invalid landing parameters");
   } else if (!PX4_ISFINITE(_disarm_delay.get()) || _disarm_delay.get()<40.f/_down_rate.get()+8.f) {
    NL_ERR("landing needs COM_DISARM_LAND long enough for controlled nose lowering (use 60 s)");
   } else if (other_hover || (modes_ready() && !in_mode(status,LAND))) {
    _x=pos.x; _y=pos.y; _yaw=e.psi(); _target_z=pos.z; _phase=Retake; _phase_start=now; _last_command=0; _was_armed=false;
    NL_INFO(modes_ready() ? "taking the hover over for landing (Land)" : "taking the hover over for landing (Offboard)");
   } else {
    _phase=Descend; _phase_start=now; _contact_dwell=0; _target_z=pos.z;
    _x=pos.x; _y=pos.y; clear_floor();
    NL_INFO("landing: descending onto rear legs");
   }
  }
  }
  if (_phase==Retake) {
   if (in_flying_mode(status,true)) {
    _was_armed=true; _phase=Descend; _phase_start=now; _contact_dwell=0; _target_z=pos.z; _x=pos.x; _y=pos.y; clear_floor();
    NL_INFO("landing: descending onto rear legs");
   } else if (now-_phase_start>5_s) {
    NL_ERR(modes_ready() ? "PX4 did not enter Land; landing cancelled" : "PX4 did not enter Offboard; landing cancelled"); _phase=Idle;
   } else if (now-_last_command>500_ms) {
    request_flying_mode(status,true); _last_command=now;
   }
  }
  if (_px4_land_takeover && (!armed || (status.nav_state!=vehicle_status_s::NAVIGATION_STATE_AUTO_LAND && !owned(status)))) {
   clear_floor(); _px4_land_takeover=false; _phase=Idle;
   if (armed) { NL_WARN("PX4 left Land: the nose is the pilot's again"); } else { NL_INFO("landing complete: disarmed"); }
   return;
  }
  if ((_phase==Idle || _phase==Failed) && armed && _enable.get() && status.nav_state==vehicle_status_s::NAVIGATION_STATE_AUTO_LAND
      && (land.ground_contact || land.maybe_landed) && !land.landed && now-att.timestamp<200_ms) {
   updateParams();
   _hover_angle=_hover_param.get();
   if (atlas_model_matches(_hover_angle, _ground) && PX4_ISFINITE(_land_angle_param.get())
       && PX4_ISFINITE(_down_rate.get()) && _down_rate.get()>=1.f && _down_rate.get()<=5.f) {
    _px4_land_takeover=true; _pilot_ground_abort=false; _pilot_handover=false; _pilot_mode=false;
    _landing_angle=_land_angle_param.get(); _rest_pitch=_landing_angle;
    _phase=LowerNose; _phase_start=now; _dwell=0; _integral=0.f; _contact_dwell=0; _touch_z=pos.z; _last_command=0;
    restart_front_slew();
    _lower_target=structural_pitch_deg(att);
    NL_INFO("PX4 Land is on the rear feet: the module lowers the nose");
   }
  }
  if ((_phase==Idle || _phase==Failed) && (in_mode(status,TO) || in_mode(status,LAND))) {
   // In its own mode with nothing to fly (Takeoff selected in the air, a landing given up): PX4 must never be left
   // without setpoints. In the air the module holds the hover where it is, as PX4's Takeoff holds once it is up (Hold
   // needs a GPS, and indoors there is none), or sinks while the commander puts its own Land back. On the ground, disarm.
   if (armed && !land.landed && in_mode(status,LAND) && !_land_fallback) {
    // Land is selected and the module is not landing (it refused): PX4's own Land takes it
    _land_fallback=true; NL_WARN("PX4's own Land takes the landing");
   }
   if (armed && !land.landed && in_mode(status,TO) && pos.xy_valid && pos.z_valid && PX4_ISFINITE(pos.x) && PX4_ISFINITE(pos.y) && PX4_ISFINITE(pos.z)) {
    _phase=Hover; _was_armed=true; _x=pos.x; _y=pos.y; _target_z=pos.z; _yaw=matrix::Eulerf(matrix::Quatf(att.q)).psi();
    _xy_reset=pos.xy_reset_counter; _z_reset=pos.z_reset_counter; _heading_reset=pos.heading_reset_counter;
    NL_INFO("holding the hover here");
   } else if (armed && !land.landed) {
    trajectory_setpoint_s sp{}; sp.timestamp=now;
    for (int i=0;i<3;i++) { sp.position[i]=NAN; sp.velocity[i]=0.f; sp.acceleration[i]=NAN; sp.jerk[i]=NAN; }
    if (_land_fallback) { sp.velocity[2]=math::constrain(_land_speed.get(),0.05f,0.3f); }
    sp.yaw=matrix::Eulerf(matrix::Quatf(att.q)).psi(); sp.yawspeed=0.f; publish_trajectory(sp,status,pos,armed);
   } else if (armed && now-_last_command>1_s) {
    command(vehicle_command_s::VEHICLE_CMD_COMPONENT_ARM_DISARM,0.f,0.f,status); _last_command=now;
   }
  }
  if (_phase==Idle || _phase==Failed) { return; }
  const bool xy_bad=!pos.xy_valid || !PX4_ISFINITE(pos.x) || !PX4_ISFINITE(pos.y) || !PX4_ISFINITE(pos.vx) || !PX4_ISFINITE(pos.vy);
  if (now-att.timestamp>200_ms || now-rates.timestamp>200_ms || now-pos.timestamp>500_ms ||
      (!_pilot_handover && !_px4_land_takeover && xy_bad) || !pos.z_valid || !PX4_ISFINITE(_pitch) || !PX4_ISFINITE(e.phi()) ||
      !PX4_ISFINITE(e.psi()) || !PX4_ISFINITE(rates.xyz[0]) || !PX4_ISFINITE(rates.xyz[1]) || !PX4_ISFINITE(rates.xyz[2]) || !PX4_ISFINITE(pos.z) || !PX4_ISFINITE(pos.vz)) {
   fail("stale or invalid estimate",status,_phase!=Prime && _phase!=Lift); return;
  }
  // Keep stored ground coordinates and setpoints in the estimator's current frame (the pilot handover uses no horizontal position).
  if (!_pilot_handover && pos.xy_reset_counter != _xy_reset) {
   if (!PX4_ISFINITE(pos.delta_xy[0]) || !PX4_ISFINITE(pos.delta_xy[1])) { fail("invalid XY reset",status,_phase!=Prime && _phase!=Lift); return; }
   _x+=pos.delta_xy[0]; _y+=pos.delta_xy[1]; _xy_reset=pos.xy_reset_counter;
  }
  if (pos.z_reset_counter != _z_reset) {
   if (!PX4_ISFINITE(pos.delta_z)) { fail("invalid height reset",status,_phase!=Prime && _phase!=Lift); return; }
   _z+=pos.delta_z; _target_z+=pos.delta_z; _touch_z+=pos.delta_z; _z_reset=pos.z_reset_counter;
  }
  if (pos.heading_reset_counter != _heading_reset) {
   if (!PX4_ISFINITE(pos.delta_heading)) { fail("invalid heading reset",status,_phase!=Prime && _phase!=Lift); return; }
   _yaw=matrix::wrap_pi(_yaw+pos.delta_heading); _heading_reset=pos.heading_reset_counter;
  }
  if (_was_armed && _pilot_handover && armed && !owned(status)) {
   // Pilot handover: leaving Offboard is the plan (Pilot, LowerNose and Shutdown run on the floor). If it happens during
   // the nose lift (stick override, mode switch), the direct front-fan hold is gone: lower the nose on the floor instead.
   if (_phase==Prime || _phase==Lift) { ground_lower(att,pos,now,"PX4 left Offboard during the nose lift",true); }
   else if (_phase==PilotGround && _handover_requested) {
    _phase=Pilot; _was_high=false; _takeover_request=false; _takeover_since=0; _handover_requested=false;
    NL_INFO("PX4 %s has the aircraft", pilot_mode_name());
   }
   else if (_phase==PilotGround) { ground_lower(att,pos,now,"PX4 left Offboard on the ground",true); }
  } else if (_was_armed && (!armed || !owned(status))) {
   clear_floor(); _was_armed=false;
   if (!armed && owned(status)) {
    // Disarmed while still under direct actuator control (typically the land detector's auto-disarm during a slow
    // nose lift): nobody publishes motor samples any more, the output driver goes quiet and lockstep SITL stops
    // advancing. Keep zero samples flowing and hand the commander back to Hold before going idle.
    NL_INFO("released control on disarm; handing PX4 back to Hold");
    _phase=Aborting; _released=true; _phase_start=now; _abort9=0.f; _abort10=0.f; _abortc=0.f; _last_command=0; return;
   }
   NL_INFO("released control on disarm or mode change"); _phase=Idle; return;
  }
  if (_pilot_mode && (_phase==Prime || _phase==Lift || _phase==Spool || _phase==Climb || _phase==Hover)) {
   if (!_pilot_gate.update(rc_valid, throttle, now)) {
    NL_WARN("pilot abort: %s (throttle %.2f)",rc_valid ? "stick held low" : "RC signal stale/lost",double(throttle));
    if (!armed) { _phase=Idle; clear_floor(); return; }
    pilot_abort(att,pos,now);
   }
  }
  if (_phase!=Pilot && (fabsf(e.phi())>math::radians(20.f) || _pitch>math::radians(15.f) || _pitch<math::radians(-55.f))) {
   fail("attitude envelope",status,_phase!=Prime && _phase!=Lift); return;   // the pilot's own flying is not policed
  }
  if (_phase==Descend) {
   // Contact inference uses PX4 estimates only and no ground datum, so the ground may differ from the takeoff spot:
   // the rear feet are down when the position controller keeps demanding descent (setpoint well below the vehicle)
   // while the vehicle has stopped moving vertically, level in roll.
   // How long that has to last is the slide on the legs: until the module takes the motors, PX4 flies on with the
   // feet down, cannot roll back to its hover trim, winds up a roll torque that the laterally canted rear jets turn
   // into a side force, and V4 slid 39 cm sideways in 0.9 s (SITL). With the range finder reading within reach of
   // what it read on the legs at takeoff, 120 ms is enough; without that, a stall in the air must not look like the
   // ground, and the contact has to hold for half a second.
   const bool rear_contact=now-_phase_start>2_s && fabsf(pos.vz)<0.055f && _target_z-pos.z>0.12f && fabsf(e.phi())<math::radians(3.f);
   const bool near_ground=PX4_ISFINITE(_park_range) && pos.dist_bottom_valid && PX4_ISFINITE(pos.dist_bottom)
       && pos.dist_bottom<_park_range+0.25f;
   if (rear_contact) {
    if (!_contact_dwell) { _contact_dwell=now; }
    if (now-_contact_dwell>(near_ground ? 120_ms : 500_ms)) {
     _phase=LowerNose; _phase_start=now; _dwell=0; _integral=0.f; _touch_z=pos.z; restart_front_slew();
     _lower_target=math::degrees(matrix::Eulerf(matrix::Dcmf(matrix::Quatf(att.q))*matrix::Dcmf(matrix::Eulerf(0.f,math::radians(_hover_angle),0.f))).theta());
     NL_INFO(near_ground ? "landing: rear support detected; lowering nose" : "landing: rear support held 0.5 s (no range check); lowering nose");
    }
   } else { _contact_dwell=0; }
   if (now-_phase_start>60_s) { fail("rear contact timeout",status,true); return; }
  }
  if (_phase==LowerNose) {
   const matrix::Dcmf structural=matrix::Dcmf(matrix::Quatf(att.q))*matrix::Dcmf(matrix::Eulerf(0.f,math::radians(_hover_angle),0.f));
   const float theta=matrix::Eulerf(structural).theta();
   const matrix::Vector3f omega=matrix::Dcmf(matrix::Eulerf(0.f,math::radians(-_hover_angle),0.f))*matrix::Vector3f(rates.xyz);
   // Rear assist (GroundFilter.hpp): near the balance point the rear motors pull the nose down, and the front fans are
   // never asked to hold it up there, so the two do not fight.
   if (_rear_assist_for!=_phase_start) {
    _rear_assist_for=_phase_start; _rear_assist=0.f; _lower_rates.reset(); _rear_assist_said=false;
   }
   _lower_rates.update(math::degrees(omega(1)), math::degrees(omega(2)));
   const float balance=AtlasRearAssist::balance_deg(_ground.gx,_ground.gz);
   _rear_assist=AtlasRearAssist::step(_rear_assist,
       AtlasRearAssist::wanted(math::degrees(theta),_lower_rates.q,balance,_rear_assist_cap.get()));
   if (_rear_assist>0.f && !_rear_assist_said) {
    _rear_assist_said=true;
    NL_INFO("lowering: nose %.1f, balance %.1f; rear motors pull it down", double(math::degrees(theta)), double(balance));
   }
   // Settle on the rear feet before starting the rate-limited pitch-down trajectory.
   if (now-_phase_start>1_s) { _lower_target=math::max(_landing_angle-3.f,_lower_target-_down_rate.get()*0.004f); }
   if (_pilot_ground_abort) { _lower_target=math::min(_lower_target,math::degrees(theta)-1.f); }
   if (PX4_ISFINITE(balance)) { _lower_target=math::min(_lower_target,balance-AtlasRearAssist::kStart); }
   const float rear_cmd=_rear_assist>0.f ? to_cmd(_rear_assist) : NAN;
   const float q_des=math::constrain(2.f*(_lower_target-math::degrees(theta)),-_down_rate.get(),_down_rate.get());
   // On the filtered rate at the lift's gain: on the raw rate at 0.02 the fans hunted at 5.6 Hz (board log 474).
   const float error=q_des-_lower_rates.q;
   _integral=math::constrain(_integral+error*0.004f,-40.f,40.f);
   const float ff=_ground.mass*9.80665f*(_ground.gx*cosf(theta)+_ground.gz*sinf(theta))/_ground.moment;
   const float frac=math::constrain(ff+0.01f*error+0.012f*_integral,0.f,1.f);
   const float delta=math::constrain(-3.f*math::radians(_lower_rates.r),-0.25f,0.25f);
   const float was9=_cmd9, was10=_cmd10, wasc=_cmdc;
   front_split(frac,delta);
   _cmd9=AtlasFrontSlew::step(was9,_cmd9,AtlasFrontSlew::kLowerUpPerSecond);
   _cmd10=AtlasFrontSlew::step(was10,_cmd10,AtlasFrontSlew::kLowerUpPerSecond);
   _cmdc=AtlasFrontSlew::step(wasc,_cmdc,AtlasFrontSlew::kLowerUpPerSecond);
   if (_pilot_ground_abort) {
    // Ground abort cannot demand more power than the pilot permitted at entry.
    // Keep per-fan limits so yaw weighting cannot increase either fan independently.
    _cmd9=AtlasPilotGate::abort_limit(_cmd9,_pilot_abort_limit9);
    _cmd10=AtlasPilotGate::abort_limit(_cmd10,_pilot_abort_limit10);
    _cmdc=AtlasPilotGate::abort_limit(_cmdc,_pilot_abort_limitc);
   }
   if (owned(status)) {
    offboard_control_mode_s mode{}; mode.timestamp=now; mode.direct_actuator=true; _mode_pub.publish(mode);
    actuator_motors_s motors{}; motors.timestamp=now; motors.timestamp_sample=att.timestamp;
    for (int i=0;i<12;i++) { motors.control[i]=NAN; }
    if (PX4_ISFINITE(rear_cmd)) { put_rear(motors.control,rear_cmd); }
    put_front(motors.control,_cmd9,_cmd10,_cmdc); _motors_pub.publish(motors);
    vehicle_thrust_setpoint_s thrust{}; thrust.timestamp=now; thrust.timestamp_sample=att.timestamp;
    thrust.xyz[2]=-front_thrust(_cmd9,_cmd10,_cmdc); _thrust_pub.publish(thrust);
   } else {
    // Pilot handover: PX4 is in the pilot's mode. Its attitude loop would fight a nose going below level (rear motors
    // up), so take the motors back through Offboard direct actuators; until PX4 switches, the floor carries the nose.
    // Under PX4's Land the floor alone carries the nose and the mode is left to PX4.
    if (!_px4_land_takeover) {
     offboard_control_mode_s mode{}; mode.timestamp=now; mode.direct_actuator=true; _mode_pub.publish(mode);
     if (now-_last_command>500_ms) { command(vehicle_command_s::VEHICLE_CMD_DO_SET_MODE,1.f,6.f,status); _last_command=now; }
    }
    publish_floor_front(now,att,_cmd9,_cmd10,_cmdc,rear_cmd);
   }
   // A stopped pitch despite a continuing nose-down request indicates nose-foot support.
   // Require the resting posture and a bounded height change since rear contact.
   // The takeoff height can drift during a long hover; use the recent contact datum.
   // XY velocity is not a nose-contact test: EKF ground-contact transients can bias it.
   const bool resting_pitch=_pilot_ground_abort ? math::degrees(theta)<=_rest_pitch+3.f : fabsf(math::degrees(theta)-_rest_pitch)<5.f;
   const bool nose_down=now-_phase_start>3_s && resting_pitch &&
       _lower_target<_rest_pitch+0.5f && q_des<-1.f && fabsf(math::degrees(omega(1)))<1.5f &&
       fabsf(pos.vz)<0.10f && fabsf(pos.z-_touch_z)<0.35f;
   if (now-_diagnostic_time>1_s) {
    _diagnostic_time=now;
    NL_INFO("lower: pitch %.2f rest %.2f q %.2f demand %.2f vz %.3f dz %.3f contact %d", double(math::degrees(theta)), double(_rest_pitch), double(math::degrees(omega(1))), double(q_des), double(pos.vz), double(pos.z-_z), int(nose_down));
   }
   if (nose_down) {
    if (!_dwell) { _dwell=now; }
    if (now-_dwell>1_s) {
     NL_INFO("landing: nose settled; ramping motors off");
     _phase=Shutdown; _phase_start=now; _abort9=_cmd9; _abort10=_cmd10; _abortc=_cmdc; _last_command=0;
    }
   } else { _dwell=0; }
   if (now-_phase_start>45_s) { fail("nose lowering timeout",status,false); }
   return;
  }
  if (_phase==Prime) {
   if (!armed && !_rng_latched) {
    // Nothing is armed and PX4's mode is not touched while this waits; the arming clock starts when it ends.
    if (range_in_use(now)) { if (!_rng_ok_since) { _rng_ok_since=now; } } else { _rng_ok_since=0; }
    if (_rng_ok_since && now-_rng_ok_since>=1_s) {
     _rng_latched=true;
     if (_rng_wait_announced) { NL_INFO("range finder in use; priming"); }
    } else {
     if (!_rng_wait_announced && now-_phase_start>1500_ms) { NL_WARN("waiting for PX4 to use the range finder before lifting the nose"); _rng_wait_announced=true; }
     if (now-_phase_start>20_s) {
      NL_ERR("PX4 is still not using the range finder (after a boot it waits for the reading to move 10 cm): lift the nose about 10 cm by hand, then press Takeoff again");
      _phase=Idle; return;
     }
     _start=now; return;
    }
   }
   if (now-_start>15_s) { fail("arming timeout",status,false); return; }
   if (now-_start>2_s && now-_last_command>1_s) {
    if (!in_flying_mode(status,false)) {
     request_flying_mode(status,false);
    } else if (!armed) { command(vehicle_command_s::VEHICLE_CMD_COMPONENT_ARM_DISARM,1.f,0.f,status); }
    _last_command=now;
   }
   if (armed && in_flying_mode(status,false)) {
    _phase=Lift; _phase_start=now; _was_armed=true; _lift_rates.reset(); _front_avg.reset(); NL_INFO("lifting nose");
   }
  }
  // In staged handoff retain the actual settled fan commands; do not remove
  // the pilot cap by recalculating an uncapped ground PI command in Spool.
  if (_phase==Lift || (_phase==Spool && !_pilot_mode)) {
   const float elapsed=(now-_phase_start)*1e-6f;
   // Structural pitch uses the configured board/hover-frame offset.
   const matrix::Dcmf structural = matrix::Dcmf(matrix::Quatf(att.q)) * matrix::Dcmf(matrix::Eulerf(0.f, math::radians(_hover_angle), 0.f));
   const float theta = matrix::Eulerf(structural).theta();
   const matrix::Vector3f omega = matrix::Dcmf(matrix::Eulerf(0.f, math::radians(-_hover_angle), 0.f)) * matrix::Vector3f(rates.xyz);
   _lift_rates.update(math::degrees(omega(1)), math::degrees(omega(2)));
   const float q = _lift_rates.q;
   const float ease = _phase==Lift ? math::constrain(elapsed / _ramp.get(), 0.f, 1.f) : 1.f;
   // Pilot handover: the nose goes all the way to PX4's level, so the attitude loop has nothing to fight when it
   // takes the sticks at low collective (a saturated pitch demand there drives rear motors up and tips the aircraft).
   const float lift_target = _pilot_handover ? _hover_angle : _lift_target.get();
   const float q_des = math::constrain(lift_target-math::degrees(theta), -_rate.get(), _rate.get()*ease);
   const float error = q_des-q;
   if (!(_pilot_mode && _phase==Lift && math::max(_cmd9,_cmd10)>=_pilot_cap-.001f && error>0.f)) { _integral=math::constrain(_integral+error*0.004f,-40.f,40.f); }
   // Feedforward from the exported current mass, rear pivot and front fan moments.
   const float ff = _ground.mass*9.80665f*(_ground.gx*cosf(theta)+_ground.gz*sinf(theta))/_ground.moment;
   // Half the rate gain the lowering uses: at 0.02 the fans hunted at 6 Hz on the aircraft (see GroundFilter.hpp).
   const float frac=math::constrain(ease*ff+0.01f*error+0.012f*_integral,0.f,1.f);
   const float delta=math::constrain(-3.f*math::radians(_lift_rates.r),-0.25f,0.25f);
   const float was9=_cmd9, was10=_cmd10, wasc=_cmdc;
   front_split(frac,delta);
   // Smooth power, not surges: each fan rises at most 0.4 per second and falls at most 2 (GroundFilter.hpp). Before
   // the pilot cap, which must still bound the result.
   _cmd9=AtlasFrontSlew::step(was9,_cmd9); _cmd10=AtlasFrontSlew::step(was10,_cmd10); _cmdc=AtlasFrontSlew::step(wasc,_cmdc);
   if (_pilot_mode && _phase==Lift) {
    // During the low-stick confirmation interval keep support instead of
    // dropping power to zero before the controlled-abort path has taken over.
    if (!_pilot_gate.low_since) {
     _pilot_cap=math::constrain(throttle,math::max(0.f,_pilot_cap-0.2f*0.004f),_pilot_cap+0.2f*0.004f);
    }
    const float largest=math::max(math::max(_cmd9,_cmd10),_cmdc);
    const float scale=largest>_pilot_cap && largest>0.f ? _pilot_cap/largest : 1.f;
    _cmd9*=scale; _cmd10*=scale; _cmdc*=scale;
   }
   _front_avg.update(to_thrust(_cmd9),to_thrust(_cmd10),_ground.fc ? to_thrust(_cmdc) : 0.f);
   if (_phase==Lift && elapsed>(_pilot_mode ? 55.f : _timeout.get())) {
    if (_pilot_mode) { NL_WARN("staged ground wait timed out: lowering with existing power limit"); pilot_abort(att,pos,now); return; }
    fail("lift timeout",status,_z-pos.z>0.3f); return;
   }
   if (_phase==Lift && _z-pos.z>0.35f && fabsf(_pitch)>math::radians(3.f)) { fail("early liftoff",status,true); return; }
   // Still means the filtered rates: the raw pitch rate carries the fans' own wobble, which never stops for 0.5 s.
   if (_phase==Lift && land.landed && fabsf(math::degrees(theta)-lift_target)<2.f && fabsf(_lift_rates.q)<AtlasGroundFilter::kSettleRate
       && fabsf(e.phi())<math::radians(3.f) && fabsf(_lift_rates.r)<3.f) {
    if (!_dwell) { _dwell=now; }
    if ((now-_dwell)*1e-6f>=_hold.get()) {
     if (_pilot_mode && !_ready_announced) { NL_INFO("nose ready: press Takeoff again for 1 m hover; lower throttle to abort"); _ready_announced=true; }
     if (_pilot_handover) {
      if (!_ready_announced) { NL_INFO("nose ready: press Takeoff again, then raise the throttle slowly (PX4 %s takes over once airborne)", pilot_mode_name()); _ready_announced=true; }
      if (second_click) {
       if (!AtlasPilotHandover::may_handover(true,true,stick_valid,stick)) { NL_WARN("handover refused: the throttle must be at minimum"); }
       else {
        _phase=PilotGround; _landing=false; _phase_start=now; _spool_z=pos.z; _lift=0.f; _liftoff_since=0; _handover_requested=false;
        _contact_dwell=0; _stick_lost_since=0; _last_command=0;
        NL_INFO("you have the throttle: raise it slowly to lift off; PX4 %s takes over once airborne", pilot_mode_name());
       }
      }
     } else if (!_pilot_mode || AtlasPilotGate::may_liftoff(true, second_click, rc_valid)) {
     _phase=Spool; _phase_start=now; _x=pos.x; _y=pos.y; _target_z=pos.z; _support_since=0; _spool_z=pos.z; _rear_fade_start=0;
     _yaw=e.psi();
     NL_INFO("nose settled and ground confirmed; climbing with heading held");
     }
    }
   } else { _dwell=0; _ready_announced=false; }
  }
  offboard_control_mode_s mode{}; mode.timestamp=now;
  if (_phase==Prime || _phase==Lift) {
   mode.direct_actuator=true; _mode_pub.publish(mode);
   actuator_motors_s motors{}; motors.timestamp=now; motors.timestamp_sample=att.timestamp;
   for (int i=0;i<actuator_motors_s::NUM_CONTROLS;i++) { motors.control[i]=NAN; }
   if (_phase==Lift) { put_front(motors.control,_cmd9,_cmd10,_cmdc); }
   _motors_pub.publish(motors);
   // Direct actuator control otherwise leaves PX4's thrust input stale. Supply the
   // requested collective thrust fraction for landing detection (not a landed flag).
   vehicle_thrust_setpoint_s thrust{}; thrust.timestamp=now; thrust.timestamp_sample=att.timestamp;
   thrust.xyz[2]=_phase==Lift ? -front_thrust(_cmd9,_cmd10,_cmdc) : 0.f;
   _thrust_pub.publish(thrust);
  } else if (_phase==PilotGround) {
   // The module flies the ground with the pilot's throttle: the rear motors follow the model's zero-torque hover
   // pattern scaled by the lift fraction, the front fans hold the nose at level with the ground model blended toward
   // their hover share as the rear motors take the weight. PX4's attitude loop stays out until the feet are off the
   // ground (on the legs, far from level, it saturates a rear motor and tips the aircraft over).
   const matrix::Dcmf structural=matrix::Dcmf(matrix::Quatf(att.q))*matrix::Dcmf(matrix::Eulerf(0.f,math::radians(_hover_angle),0.f));
   const float theta=matrix::Eulerf(structural).theta();
   const matrix::Vector3f omega=matrix::Dcmf(matrix::Eulerf(0.f,math::radians(-_hover_angle),0.f))*matrix::Vector3f(rates.xyz);
   if (stick_valid) { _stick_lost_since=0; } else if (!_stick_lost_since) { _stick_lost_since=now; }
   if (_stick_lost_since && now-_stick_lost_since>2_s && !_handover_requested) { ground_lower(att,pos,now,"remote lost",true); return; }
   const float want=stick_valid ? AtlasPilotHandover::lift_fraction(stick,_lmax.get()) : 0.f;
   _lift=math::constrain(want,_lift-2.f*0.004f,_lift+1.f*0.004f);   // at most +1/s up and -2/s down: no thrust steps
   const float rise=_spool_z-pos.z;
   const float q_des=math::constrain(_hover_angle-math::degrees(theta),-_rate.get(),_rate.get());
   const float error=q_des-math::degrees(omega(1));
   _integral=math::constrain(_integral+error*0.004f,-40.f,40.f);
   const float ff=_ground.mass*9.80665f*(_ground.gx*cosf(theta)+_ground.gz*sinf(theta))/_ground.moment;
   const float base=(_lift<1.f) ? ff*(1.f-_lift)+_lift*_uf.get() : _uf.get()*_lift;
   const float frac=math::constrain(base+0.02f*error+0.012f*_integral,0.f,1.f);
   const float delta=math::constrain(-3.f*omega(2),-0.25f,0.25f);
   front_split(frac,delta);
   mode.direct_actuator=true; _mode_pub.publish(mode);
   actuator_motors_s motors{}; motors.timestamp=now; motors.timestamp_sample=att.timestamp;
   for (int i=0;i<actuator_motors_s::NUM_CONTROLS;i++) { motors.control[i]=NAN; }
   // the hover pattern is the ten-motor layout's (the takeoff request refuses the pilot handover on any other)
   const float pattern[8]={_u1.get(),_u2.get(),_u3.get(),_u4.get(),_u5.get(),_u6.get(),_u7.get(),_u8.get()};
   for (int i=0;i<8;i++) { motors.control[i]=to_cmd(pattern[i]*_lift); }
   put_front(motors.control,_cmd9,_cmd10,_cmdc); _motors_pub.publish(motors);
   vehicle_thrust_setpoint_s thrust{}; thrust.timestamp=now; thrust.timestamp_sample=att.timestamp;
   thrust.xyz[2]=-math::constrain(0.6f*_lift,0.f,1.f); _thrust_pub.publish(thrust);
   if (!_landing) {
    if (!_handover_requested) {
     if (AtlasPilotHandover::liftoff(rise,_lift)) { if (!_liftoff_since) { _liftoff_since=now; } } else { _liftoff_since=0; }
     if (_liftoff_since && now-_liftoff_since>=AtlasPilotHandover::kLiftoffDwell) {
      _handover_requested=true; _last_command=0; NL_INFO("airborne: handing PX4 %s the aircraft", pilot_mode_name());
     }
     if (now-_phase_start>AtlasPilotHandover::kGroundWait) { ground_lower(att,pos,now,"no liftoff within 45 s",true); return; }
    }
    if (_handover_requested) {
     if (now-_last_command>500_ms) {
      command(vehicle_command_s::VEHICLE_CMD_DO_SET_MODE,1.f,_pilot_nav==vehicle_status_s::NAVIGATION_STATE_ALTCTL ? 2.f : 7.f,status); _last_command=now;
     }
     if (now-_liftoff_since>5_s && now-_diagnostic_time>5_s) { _diagnostic_time=now; NL_WARN("PX4 has not taken the aircraft; switch to %s by hand", pilot_mode_name()); }
    }
   } else {
    // Landing: the pilot lowers the throttle. The rear feet are down when the descent stalls at low throttle.
    if (AtlasPilotHandover::contact_sample(stick_valid,stick,pos.vz)) { if (!_contact_dwell) { _contact_dwell=now; } } else { _contact_dwell=0; }
    if (_contact_dwell && now-_contact_dwell>=AtlasPilotHandover::kContactDwell) {
     _phase=LowerNose; _phase_start=now; _dwell=0; _integral=0.f; _contact_dwell=0; _touch_z=pos.z; _pilot_ground_abort=false;
     _lower_target=math::degrees(theta);
     NL_INFO("landing: rear support detected; lowering nose"); return;
    }
    if (rise>AtlasPilotHandover::kHandBackRise && _lift>1.f) {
     // climbed away again: give PX4 the aircraft back
     _landing=false; _handover_requested=true; _liftoff_since=now; _last_command=0; NL_INFO("landing abandoned: handing PX4 %s the aircraft", pilot_mode_name());
    }
   }
  } else if (_phase==Pilot) {
   // PX4 flies for the pilot; the module only watches for the landing takeover (Land press, or low and descending
   // with the throttle at or below the centre once the aircraft has been up).
   const float rise=_spool_z-pos.z;
   if (rise>AtlasPilotHandover::kHighLatch) { _was_high=true; }
   if (_takeover_request || AtlasPilotHandover::takeover(_was_high,stick_valid,stick,rise,pos.vz)) {
    if (!_takeover_since) { _takeover_since=now; _last_command=0; NL_INFO("taking the landing over: lower the throttle to descend"); }
    mode.direct_actuator=true; _mode_pub.publish(mode);   // the offboard signal PX4 requires before it grants Offboard
    if (now-_last_command>500_ms) { command(vehicle_command_s::VEHICLE_CMD_DO_SET_MODE,1.f,6.f,status); _last_command=now; }
    if (now-_takeover_since>5_s && now-_diagnostic_time>5_s) { _diagnostic_time=now; NL_WARN("PX4 has not entered Offboard; keep flying, or land in %s", pilot_mode_name()); }
   } else { _takeover_since=0; }
   if (status.nav_state==vehicle_status_s::NAVIGATION_STATE_OFFBOARD) {
    _phase=PilotGround; _landing=true; _phase_start=now; _integral=0.f; _contact_dwell=0; _stick_lost_since=0;
    _takeover_request=false; _takeover_since=0; _handover_requested=false;
    _lift=stick_valid ? AtlasPilotHandover::lift_fraction(stick,_lmax.get()) : 1.f;
    NL_INFO("landing: you have the throttle; lower it to descend onto the rear legs");
   }
  } else {
   if (_phase==Spool) {
    atlas_nose_lift_floor_s output{}; _motors_sub.copy(&output);
    // Keep the output stream alive until the position-control allocator has published.
    // Without this bridge lockstep waits for motors while commander waits for time
    // to advance before activating the new control mode.
    if (!output.active || output.timestamp < _phase_start) {
     actuator_motors_s bridge{}; bridge.timestamp=now; bridge.timestamp_sample=att.timestamp;
     for (int i=0;i<12;i++) { bridge.control[i]=NAN; }
     put_front(bridge.control,_cmd9,_cmd10,_cmdc);
     _motors_pub.publish(bridge);
    }
    // Compare unmodified allocation against support, avoiding feedback through the floor. A momentary match is not
    // enough: right after the mode switch the attitude loop alone can put the front fans above the floor while the
    // takeoff ramp still has the rear motors at zero, and fading then drops the nose and skids the aircraft forward.
    // Require a continuous 150 ms allocation match before fading the front support.
    const bool supported = output.active && now-output.timestamp<100_ms && output.control[_ground.fa-1]>=0.95f*_cmd9
        && output.control[_ground.fb-1]>=0.95f*_cmd10 && (!_ground.fc || output.control[_ground.fc-1]>=0.95f*_cmdc);
    if (!supported) { _support_since=0; } else if (!_support_since) { _support_since=now; }
    if (!_fade_start && supported && now-_support_since>=150_ms) {
     _fade_start=now; _fade9=_cmd9; _fade10=_cmd10; _fadec=_cmdc; NL_INFO("PX4 supports nose and is lifting; fading motor floor");
    }
    if (now-_phase_start>15_s && !_fade_start) { fail("handover timeout",status,!land.landed); return; }
    atlas_nose_lift_floor_s floor{}; floor.timestamp=now; floor.timestamp_sample=att.timestamp; floor.active=true;
    for(int i=0;i<12;i++) { floor.control[i]=NAN; }
    // Rear motors: while the rear feet touch, the aircraft cannot tilt to counter the front fans' rearward push and
    // slides on its feet. Floor the rear motors with a fast ramp so the feet leave the ground at once; release the
    // floor a second after the aircraft is airborne (PX4 is in charge by then). Roll stays neutral: all rear equal.
    if (!_rear_fade_start && _spool_z-pos.z>0.10f) { _rear_fade_start=now; NL_INFO("airborne; releasing the rear floor"); }
    const float rear_ramp=math::constrain((now-_phase_start)*1e-6f/math::max(0.2f,_spool_t.get()),0.f,1.f);
    // The front floor was only holding the nose while the rear feet carried the weight: hand the nose to PX4's
    // attitude loop as the rear lifts (a held front floor pins the nose up and PX4 cannot lower it).
    const float front_scale=(_spool_cmd.get()>0.f) ? (1.f-rear_ramp) : 1.f;
    const float fade = _fade_start ? math::constrain(1.f-(now-_fade_start)*1e-6f/1.f,0.f,1.f) : 1.f;
    put_front(floor.control,(_fade_start ? math::min(_cmd9,_fade9*fade) : _cmd9)*front_scale,
              (_fade_start ? math::min(_cmd10,_fade10*fade) : _cmd10)*front_scale,
              (_fade_start ? math::min(_cmdc,_fadec*fade) : _cmdc)*front_scale);
    const float rear_fade=_rear_fade_start ? math::constrain(1.f-(now-_rear_fade_start)*1e-6f/1.f,0.f,1.f) : 1.f;
    const float rear=math::constrain(_spool_cmd.get(),0.f,1.f)*rear_ramp*rear_fade;
    if (rear>0.f) { for (int i=0;i<math::min((int)_ground.count,12);i++) { if (!is_front(i)) { floor.control[i]=rear; } } }
    _floor_pub.publish(floor);
    if (_fade_start && now-_fade_start>1_s) { _phase=Climb; _phase_start=now; NL_INFO("handover complete"); }
   }
   mode.position=true; _mode_pub.publish(mode);
   // The climb is a moving target, and PX4 is told how fast it moves. With a position alone its controller aims to stop
   // on the target: right after liftoff the aircraft caught up with the rising target while still climbing, PX4 cut
   // the thrust to brake on it and the aircraft sagged 0.3 m (SITL, V4 at 17.8 deg). As in PX4's own takeoff, the
   // setpoint now carries the target's vertical velocity: 0.5 m/s up, easing to zero at 0.5 m/s^2 over the last
   // quarter metre. The hover keeps a plain position.
   float climb_v=0.f;
   if (_phase==Descend) { _target_z=math::min(pos.z+0.5f,_target_z+_land_speed.get()*0.004f); }   // keep descending, at most 0.5 m below the vehicle
   // Request ascent immediately on entering Spool, including while the rear feet touch.
   // Holding measured height here prevents PX4's takeoff ramp from receiving climb demand
   // when the optional rear floor is off. Preserve this trajectory through Spool -> Climb.
   else if (_phase!=Retake) {
    const float left=math::max(_target_z-(_z-_flight_alt),0.f);   // metres still to climb (NED: up is negative)
    climb_v=math::min(0.5f,sqrtf(2.f*0.5f*left));
    _target_z=math::max(_z-_flight_alt,_target_z-climb_v*0.004f);
   }
   trajectory_setpoint_s sp{}; sp.timestamp=now;
   sp.position[0]=_x; sp.position[1]=_y; sp.position[2]=_target_z;
   for(int i=0;i<3;i++) { sp.velocity[i]=NAN; sp.acceleration[i]=NAN; sp.jerk[i]=NAN; }
   if (climb_v>0.f) { sp.velocity[2]=-climb_v; }
   sp.yaw=_yaw; sp.yawspeed=0.f; publish_trajectory(sp,status,pos,armed);
   if (_phase==Climb && fabsf(pos.z-(_z-_flight_alt))<0.15f && fabsf(pos.vz)<0.15f) {
    _phase=Hover; NL_INFO("hover achieved with heading held");
   }

   if (_phase==Climb && now-_phase_start>30_s) { fail("climb timeout",status,true); }
  }
 }
};
ModuleBase::Descriptor AtlasNoseLift::desc{task_spawn,custom_command,print_usage};
extern "C" __EXPORT int atlas_nose_lift_main(int argc,char *argv[]) { return ModuleBase::main(AtlasNoseLift::desc,argc,argv); }
