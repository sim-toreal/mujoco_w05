//
// Created by bohm on 15/12/22.
//

#ifndef W05_SWING_CONTROLLER_H
#define W05_SWING_CONTROLLER_H
#pragma once

#include <mujoco/mujoco.h>

namespace SwingController {
    const int fsm_hold = 0;
    const int fsm_swing1 = 1;
    const int fsm_swing2 = 2;
    const int fsm_end = 3;

    const double t_hold = 0.5;
    const double t_swing1 = 1;
    const double t_swing2 = 1;

    extern int fsm_state;

    void init_controller(const mjModel* m, mjData* d);

    void controller(const mjModel* m, mjData* d);
}

#endif //W05_SWING_CONTROLLER_H
