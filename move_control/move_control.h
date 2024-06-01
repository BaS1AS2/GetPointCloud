#ifndef MOVE_CONTROL_H
#define MOVE_CONTROL_H

#endif  // MOVE_CONTROL_H

#include <qdebug.h>
#include <windows.h>

#include <iostream>
#include <thread>

#include "lib/zauxdll2.h"
#include "lib/zmotion.h"

namespace MoveControl {

void move(ZMC_HANDLE handle, int axis_num, int axis[], float longth[]);

void safe(ZMC_HANDLE handle, int axis[], float longth);

}  // namespace MoveControl
