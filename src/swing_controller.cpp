//
// Created by bohm on 15/12/22.
//
#include "swing_controller.h"

int SwingController::fsm_state = fsm_hold;
SwingController::A SwingController::a;

void SwingController::init_controller(const mjModel *m, mjData *d) {
  fsm_state = fsm_hold;

}

void SwingController::controller(const mjModel *m, mjData *d) {
  double t = d->time;

  // transitions
  if (fsm_state == fsm_hold && d->time >= t_hold) {
    fsm_state = fsm_swing1;
    double q_0[2] = {-1, 0}, q_f[2] = {0.5, -2};
    a = SwingController::generate_trajectory(t_hold, t_hold + t_swing1, q_0, q_f);
  }
  if (fsm_state == fsm_swing1 && d->time >= t_hold + t_swing1) {
    fsm_state = fsm_swing2;
    double q_0[2] = {0.5, -2}, q_f[2] = {1, 0};
    a = SwingController::generate_trajectory(t_hold + t_swing1, t_hold + t_swing1 + t_swing2, q_0, q_f);
  }
  if (fsm_state == fsm_swing2 && d->time >= t_hold + t_swing1 + t_swing2) {
    fsm_state = fsm_end;
  }

  double qref[2] = {0}, uref[2] = {0};
  if (fsm_state == fsm_swing1 || fsm_state == fsm_swing2) {
    for (int i = 0; i < 2; ++i) {
      qref[i] = a.a0[i] + a.a1[i] * t + a.a2[i] * t * t + a.a3[i] * t * t * t;
      uref[i] = a.a1[i] + 2 * a.a2[i] * t + 3 * a.a3[i] * t * t;
    }
  }

  // Actions
  // start q0 = -1; q1 = 0
  // intermediate q0 = 0, q1 = -1.57 (pi/2)
  // end: q0 = 1; q1 = 0
  double kp = 500, kv = 50;
  double q0, q1;
  if (fsm_state == fsm_hold) {
    q0 = -1;
    q1 = 0;
    d->ctrl[0] = -kp * (d->qpos[0] - q0) - kv * d->qvel[0];
    d->ctrl[1] = -kp * (d->qpos[1] - q1) - kv * d->qvel[1];
  }
  if (fsm_state == fsm_swing1 || fsm_state == fsm_swing2) {
    d->ctrl[0] = -kp*(d->qpos[0]-qref[0])-kv*(d->qvel[0]-uref[0]);
    d->ctrl[1] = -kp*(d->qpos[1]-qref[1])-kv*(d->qvel[1]-uref[1]);
  }
  if (fsm_state == fsm_end) {
    q0 = 1;
    q1 = 0;
    d->ctrl[0] = -kp * (d->qpos[0] - q0) - kv * d->qvel[0];
    d->ctrl[1] = -kp * (d->qpos[1] - q1) - kv * d->qvel[1];
  }

  std::cout << d->ctrl[0] << ", " << d->ctrl[1] << std::endl;
//  d->ctrl[1] = -5;

  // this is using position actuators, but it hits the floor
  //    d->ctrl[1] = q0;
//    d->ctrl[4] = q1;

//  d->ctrl[0] = -kp * (d->qpos[0] - q0) - kv * d->qvel[0];
//  d->ctrl[3] = -kp * (d->qpos[1] - q1) - kv * d->qvel[1];

}

SwingController::A SwingController::generate_trajectory(double t0, double tf, double q_0[2], double q_f[2]) {
  A a{};
  double tf_t0_3 = (tf-t0)*(tf-t0)*(tf-t0);
  for (int i = 0; i < 2; ++i) {
    double q0 = q_0[i], qf = q_f[i];
    a.a0[i] = qf*t0*t0*(3*tf-t0) + q0*tf*tf*(tf-3*t0);
    a.a0[i] = a.a0[i]/tf_t0_3;
    a.a1[i] = 6*t0*tf*(q0-qf);
    a.a1[i] = a.a1[i]/tf_t0_3;
    a.a2[i] = 3*(t0+tf)*(qf-q0);
    a.a2[i] = a.a2[i]/tf_t0_3;
    a.a3[i] = 2*(q0-qf);
    a.a3[i] = a.a3[i]/tf_t0_3;
  }

  return a;
}