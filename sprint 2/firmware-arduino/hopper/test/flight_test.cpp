// Host test of the flight phases (control/flight.cpp) flying the real controllers on a 1-D
// vehicle model. No hardware needed. From firmware-arduino/hopper:
//   c++ -std=c++17 -Itest/stub -Isrc -o /tmp/flight_test test/flight_test.cpp \
//       src/control/flight.cpp src/control/controllers.cpp src/control/ctrl_pd.cpp src/control/ctrl_cascade.cpp \
//     && /tmp/flight_test
// The Arduino build ignores this folder (it only compiles the sketch folder and src/).
#include "Arduino.h"
#include <initializer_list>
#include "control/flight.h"
#include "control/controllers.h"
#include "protocol.h"
static int fails = 0;
#define CHECK(c, msg) do { if (!(c)) { ::printf("  FAIL: %s\n", msg); fails++; } else ::printf("  ok: %s\n", msg); } while (0)
static const char* PH[] = {"IDLE","ASCENT","HOVER","DESCENT","LANDED","ABORT"};

struct Run { uint8_t phases_seen[16]; int nph=0; float zmax=0, vtouch=0, max_lag=0; uint8_t final_phase, abort_r; float t_end; };

struct Opts { uint8_t mode=CTRL_PD_POS; int16_t hop=500; uint16_t up=250, down=200, hover=3000;
  bool go_at_arm=false; float drop_go_at=-1, nav_lost_at=-1, servo_lost_at=-1, power_trip_at=-1;
  float thrust_scale=1.0f; float T=30; bool no_ff=false; };

Run fly(const Opts& o, const char* name) {
  ::printf("== %s\n", name);
  Run r; float z=0, vz=0, valve=0; const float g=9.80665f, dt=0.004f;
  flight_reset(); CONTROLLERS[o.mode].reset();
  uint8_t last = 255; bool airborne=false;
  for (int k=0; k*dt < o.T; k++) {
    float t=k*dt; uint32_t now=(uint32_t)(t*1000)+1000;
    flight_inputs_t in{}; in.z=z; in.vz=vz;
    in.go = o.go_at_arm ? true : (t>=0.5f && !(o.drop_go_at>0 && t>=o.drop_go_at));
    in.closed_loop = o.mode!=CTRL_MANUAL;
    in.nav_ok = !(o.nav_lost_at>0 && t>=o.nav_lost_at);
    in.health = HEALTH_IMU|HEALTH_LASER|HEALTH_PT|(o.servo_lost_at>0 && t>=o.servo_lost_at ? 0 : HEALTH_SERVO_PWR);
    in.power_trip = o.power_trip_at>0 && t>=o.power_trip_at;
    in.request = flight_clamp_profile(o.hop,o.up,o.down,o.hover,&in.request_clamped);
    uint8_t was = flight_phase();
    flight_ref_t ref = flight_update(now,&in);
    uint8_t u=0;
    if (ref.action==FLIGHT_TRACK) { if (was==PHASE_IDLE) CONTROLLERS[o.mode].reset();
      ctrl_in_t c{}; c.dt=dt; c.z=z; c.vz=vz; c.az=0; c.z_sp=ref.z_sp; c.vz_sp=o.no_ff?0:ref.vz_sp;
      u=controller_run(o.mode,&c);
      if (flight_phase()==PHASE_ASCENT && ref.vz_sp>0 && t>1.5f) r.max_lag=fmaxf(r.max_lag, ref.z_sp-z); }
    else if (ref.action==FLIGHT_MANUAL) u=0;
    if (flight_phase()!=last) { last=flight_phase(); r.phases_seen[r.nph++]=last; }
    // plant
    valve += (u-valve)*dt/0.05f;
    float a = g*o.thrust_scale*valve/CTRL_DEFAULT_GAINS.hover_pct - g;
    vz += a*dt; z += vz*dt;
    if (z<=0) { if (airborne && vz<0 && r.vtouch==0) r.vtouch=-vz; z=0; if (vz<0) vz=0; }
    if (z>0.02f) airborne=true;
    r.zmax=fmaxf(r.zmax,z);
    if (flight_phase()==PHASE_LANDED && z==0 && vz==0) { r.t_end=t; break; }
    r.t_end=t;
  }
  r.final_phase=flight_phase(); r.abort_r=flight_abort_reason();
  ::printf("  phases:"); for(int i=0;i<r.nph;i++) ::printf(" %s",PH[r.phases_seen[i]]);
  ::printf(" | zmax %.3f m, touchdown %.2f m/s, climb lag %.3f m, abort %u, t %.1f s\n", r.zmax, r.vtouch, r.max_lag, r.abort_r, r.t_end);
  return r;
}
static bool seq(const Run& r, std::initializer_list<int> want){ int i=0; for(int w:want){ if(i>=r.nph||r.phases_seen[i]!=w) return false; i++; } return i==r.nph; }

int main(){
  Opts o; Run r;
  r=fly(o,"nominal hop, PD");
  CHECK(seq(r,{PHASE_IDLE,PHASE_ASCENT,PHASE_HOVER,PHASE_DESCENT,PHASE_LANDED}),"IDLE>ASCENT>HOVER>DESCENT>LANDED");
  CHECK(r.zmax<0.5f+0.10f && r.zmax>0.45f,"peak within 10 cm of 0.5 m");
  CHECK(r.max_lag<0.08f,"climb lag under 8 cm (feedforward)");
  { Opts n; n.no_ff=true; Run q=fly(n,"same, PD WITHOUT feedforward (comparison)"); CHECK(q.max_lag>2*r.max_lag,"feedforward at least halves the lag"); }
  CHECK(r.vtouch<0.30f,"touchdown slower than 0.3 m/s");
  CHECK(r.abort_r==ABORT_NONE,"no abort");

  o=Opts(); o.mode=CTRL_CASCADE; r=fly(o,"nominal hop, cascade");
  CHECK(seq(r,{PHASE_IDLE,PHASE_ASCENT,PHASE_HOVER,PHASE_DESCENT,PHASE_LANDED}),"full profile");
  CHECK(r.max_lag<0.05f,"climb lag under 5 cm");
  CHECK(r.vtouch<0.30f,"touchdown slower than 0.3 m/s");

  o=Opts(); o.go_at_arm=true; o.T=3; r=fly(o,"go already on at arm");
  CHECK(r.final_phase==PHASE_IDLE && r.zmax==0,"no hop started");

  o=Opts(); o.mode=CTRL_MANUAL; o.T=3; r=fly(o,"go with MANUAL controller");
  CHECK(r.final_phase==PHASE_IDLE && r.abort_r==ABORT_MANUAL,"refused, reason MANUAL");

  o=Opts(); o.nav_lost_at=0.3f; o.T=3; r=fly(o,"nav lost before go");
  CHECK(r.final_phase==PHASE_IDLE && r.abort_r==ABORT_PREFLIGHT,"refused, reason pre-flight");

  o=Opts(); o.drop_go_at=4.0f; r=fly(o,"go dropped in HOVER");
  CHECK(r.final_phase==PHASE_LANDED && r.abort_r==ABORT_GROUND,"ABORT then LANDED, reason go dropped");
  CHECK(r.phases_seen[r.nph-2]==PHASE_ABORT,"went through ABORT (controlled descent)");
  CHECK(r.vtouch<0.5f,"abort touchdown slower than 0.5 m/s");

  o=Opts(); o.nav_lost_at=1.5f; r=fly(o,"nav lost in ASCENT");
  CHECK(r.final_phase==PHASE_LANDED && r.abort_r==ABORT_SENSOR && r.phases_seen[r.nph-2]==PHASE_ASCENT,"straight to LANDED (valve cut)");

  o=Opts(); o.servo_lost_at=2.0f; r=fly(o,"servo power lost");
  CHECK(r.final_phase==PHASE_LANDED && r.abort_r==ABORT_SERVO,"LANDED, reason servo");

  o=Opts(); o.power_trip_at=4.0f; r=fly(o,"power protection trips in HOVER");
  CHECK(r.final_phase==PHASE_LANDED && r.abort_r==ABORT_POWER,"LANDED, reason power");
  CHECK(r.phases_seen[r.nph-2]==PHASE_ABORT,"went through ABORT (controlled descent, not a cut)");
  CHECK(r.vtouch<0.5f,"power-trip touchdown slower than 0.5 m/s");

  o=Opts(); o.power_trip_at=0.2f; o.T=3; r=fly(o,"power protection tripped before go");
  CHECK(r.final_phase==PHASE_IDLE && r.abort_r==ABORT_POWER,"refused, reason power");

  o=Opts(); o.power_trip_at=7.0f; r=fly(o,"power protection trips in DESCENT");
  CHECK(seq(r,{PHASE_IDLE,PHASE_ASCENT,PHASE_HOVER,PHASE_DESCENT,PHASE_LANDED}),"keeps its normal descent");

  o=Opts(); o.hop=2000; o.up=5000; o.down=9; o.hover=60000; o.T=0.6f;
  { flight_reset(); bool cl; flight_profile_t p=flight_clamp_profile(o.hop,o.up,o.down,o.hover,&cl);
    ::printf("== clamping\n"); CHECK(cl && p.hop_m==0.8f && p.v_up==0.5f && p.v_down==0.05f && p.hover_ms==10000,"2 m/5 m/s/9 mm/s/60 s clamped to 0.8/0.5/0.05/10 s"); }

  o=Opts(); o.thrust_scale=0.8f; o.T=40; r=fly(o,"too weak to reach the hop height");
  CHECK(r.final_phase==PHASE_LANDED && r.abort_r==ABORT_TIMEOUT,"timeout ABORT, then LANDED");

  o=Opts(); o.thrust_scale=1.6f; o.drop_go_at=3.0f; o.T=30; r=fly(o,"ABORT that cannot come down (60% excess thrust)");
  CHECK(r.final_phase==PHASE_LANDED,"valve cut after the ABORT landing timeout");
  ::printf("\n%s (%d failures)\n", fails? "FAILED":"ALL PASSED", fails); return fails!=0;
}
