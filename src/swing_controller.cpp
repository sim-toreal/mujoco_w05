//
// Created by bohm on 15/12/22.
//
#include "swing_controller.h"

int SwingController::fsm_state = fsm_hold;

void SwingController::init_controller(const mjModel *m, mjData *d) {
  fsm_state = fsm_hold;

}

void SwingController::controller(const mjModel *m, mjData *d) {

  // transitions
  if (fsm_state == fsm_hold && d->time >= t_hold) {
    fsm_state = fsm_swing1;
  }
  if (fsm_state == fsm_swing1 && d->time >= t_hold + t_swing1) {
    fsm_state = fsm_swing2;
  }
  if (fsm_state == fsm_swing2 && d->time >= t_hold + t_swing1 + t_swing2) {
    fsm_state = fsm_end;
  }

  // Actions
  // start q0 = -1; q1 = 0
  // intermediate q0 = 0, q1 = -1.57 (pi/2)
  // end: q0 = 1; q1 = 0
  double kp = 1000, kv = 100;
  double q0, q1;
  if (fsm_state == fsm_hold) {
    q0 = -1;
    q1 = 0;
  }
  if (fsm_state == fsm_swing1) {
    q0 = 0;
    q1 = -1.57;
  }
  if (fsm_state == fsm_swing2) {
    q0 = 1;
    q1 = 0;
  }
  if (fsm_state == fsm_end) {
    q0 = 1;
    q1 = 0;
  }

  // this is using position actuators, but it hits the floor
  //    d->ctrl[1] = q0;
//    d->ctrl[4] = q1;

  d->ctrl[0] = -kp * (d->qpos[0] - q0) - kv * d->qvel[0];
  d->ctrl[1] = -kp * (d->qpos[1] - q1) - kv * d->qvel[1];

}