//
// Created by bohm on 15/12/22.
//

#ifndef W05_SWING_CONTROLLER_H
#define W05_SWING_CONTROLLER_H

#include <mujoco/mujoco.h>

namespace SwingController {
    void controller(const mjModel* m, mjData* d);
}

#endif //W05_SWING_CONTROLLER_H
