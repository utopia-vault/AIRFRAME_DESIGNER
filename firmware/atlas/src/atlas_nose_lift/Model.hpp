#pragma once
#include <parameters/param.h>
#include <cmath>
// The ground model the exporter computes from the design: mass, CG about the rear-foot pivot, the front fans' nose-up
// authority and which motors they are. A, B are the yaw pair (A takes w9 of the nose effort, B 2 - w9); C is an optional
// third front fan with weight wc. The ten-motor layout is A = 9, B = 10, no C.
struct AtlasGroundModel {
 float mass{}, gx{}, gz{}, moment{}, w9{}, wc{}, thr_fac{};
 int32_t count{10}, fa{9}, fb{10}, fc{0};
 // the layout the pilot handover's hover pattern (NLF_U1..8 rear, NLF_UF front) describes
 bool ten_motor_pair() const { return count==10 && fa==9 && fb==10 && fc==0; }
};
inline bool atlas_model_matches(float hover, AtlasGroundModel &m) {
 int32_t enabled{}, count{};
 param_get(param_find("NLF_CFG_OK"), &enabled);
 param_get(param_find("CA_ROTOR_COUNT"), &count);
 if (enabled!=1 || count<3 || count>12 || !std::isfinite(hover)) {
  PX4_ERR("unsupported ground geometry; Update PX4 with a tripod model the exporter accepts"); return false;
 }
 const char *names[]={"NLF_MASS","NLF_GX","NLF_GZ","NLF_MOM","NLF_W9","NLF_WC"};
 float *values[]={&m.mass,&m.gx,&m.gz,&m.moment,&m.w9,&m.wc};
 for (int i=0;i<6;i++) {
  param_t p=param_find(names[i]);
  if(p==PARAM_INVALID || param_get(p,values[i])!=0 || !std::isfinite(*values[i])) { return false; }
 }
 const char *slots[]={"NLF_FA","NLF_FB","NLF_FC"};
 int32_t *motors[]={&m.fa,&m.fb,&m.fc};
 for (int i=0;i<3;i++) {
  param_t p=param_find(slots[i]);
  if(p==PARAM_INVALID || param_get(p,motors[i])!=0) { return false; }
 }
 if (!(m.fa>=1 && m.fa<=count && m.fb>=1 && m.fb<=count && m.fa!=m.fb &&
       (m.fc==0 || (m.fc>=1 && m.fc<=count && m.fc!=m.fa && m.fc!=m.fb)))) {
  PX4_ERR("NLF_FA/FB/FC must name distinct motors within CA_ROTOR_COUNT; Update PX4"); return false;
 }
 m.count=count;
 // The fans are quadratic and PX4 inverts THR_MDL_FAC on its way out, so the module can work with any factor: a thrust
 // fraction t is the command (1 - f) sqrt(t) + f t.
 float factor{}; param_t p=param_find("THR_MDL_FAC");
 if(p==PARAM_INVALID || param_get(p,&factor)!=0 || !std::isfinite(factor) || factor<0.f || factor>1.f) { PX4_ERR("THR_MDL_FAC must be within 0..1"); return false; }
 m.thr_fac=factor;
 if (!(m.mass>0.f && m.gx>0.f && m.moment>0.f && m.w9>0.f && m.w9<2.f && m.wc>=0.f && m.wc<2.f && (m.fc==0)==(m.wc<=0.f))) {   // a third fan exactly when it has a share
  PX4_ERR("NLF_MASS/GX/MOM/W9/WC out of range; Update PX4"); return false;
 }
 return true;
}
