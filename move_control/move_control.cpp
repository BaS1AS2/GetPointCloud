#include "move_control.h"

namespace MoveControl {

void move(ZMC_HANDLE handle, int axis_num, int axis[], float longth[]) {
    int status[3];
    // ZAux_Direct_MoveSynmove(handle, axis, axis - 1, 50, 0);
    int ret = ZAux_Direct_MoveAbs(handle, axis_num, axis, longth);  // 轴当前位置运动x个units
    // commandCheckHandler("ZAux_Direct_MoveAbs", ret);
    // ZAux_Direct_MoveASynmove(handle, axis, axis - 1, 50, 0);
    //  判断运动状态
    do {
        for (int i = 0; i < 3; i++) {
            ret = ZAux_Direct_GetIfIdle(handle, axis[i],
                                        &status[i]);  // 读取轴运动状态， 0-运动， 1-停止
            // commandCheckHandler("ZAux_Direct_GetIfIdle", ret);
        }
        if (!(status[0] || status[1] || status[2])) {
            // printf("axis is running!\n");
            float position[3];
            // 获取当前轴位置,如果超过量程则直接停止
            for (int i = 0; i < 3; i++) {
                ZAux_Direct_GetDpos(handle, axis[i], &position[i]);
                // std::cout << "position is:" << position[0]<<" "<< position[1]<<" "<<
                // position[2]<< std::endl;
            }
            if (*longth > 0) {
                for (int i = 0; i < 3; i++) {
                    if (position[i] > 250) {
                        ret = ZAux_Direct_Single_Cancel(handle, axis[i], 3);
                        // commandCheckHandler("ZAux_Direct_Single_Cancel", ret);
                    }
                }

            } else {
                for (int i = 0; i < 3; i++) {
                    if (position[i] < 0) {
                        ret = ZAux_Direct_Single_Cancel(handle, axis[i], 3);
                        // commandCheckHandler("ZAux_Direct_Single_Cancel", ret);
                    }
                }
            }
        }
        Sleep(100);
    } while ((status[0] && status[1] && status[2]) == 0);  // 等待轴停止
    if (ret == 0) {
        qDebug() << "axis is not running\n";
    } else {
        printf("error has occured,stop detect axis status\n");
    }
    // 到位后停止
    for (int i = 0; i < 3; i++) {
        ret = ZAux_Direct_Single_Cancel(handle, axis[i], 3);
        // commandCheckHandler("ZAux_Direct_Single_Cancel", ret);
    }
}

void safe(ZMC_HANDLE handle, int axis[], float longth) {
    // 限位控制
    float position[3];
    // 获取当前轴位置,如果超过量程则直接停止
    for (int i = 0; i < 3; i++) {
        ZAux_Direct_GetDpos(handle, axis[i], &position[i]);
        // std::cout << "position is:" << position[0]<<" "<< position[1]<<" "<< position[2]<<
        // std::endl;
    }
    if (longth > 0) {
        for (int i = 0; i < 3; i++) {
            if (position[i] > 250) {
                //                int ret = ZAux_Direct_Single_Cancel(handle, axis[i], 3);
                // commandCheckHandler("ZAux_Direct_Single_Cancel", ret);
            }
        }

    } else {
        for (int i = 0; i < 3; i++) {
            if (position[i] < 0) {
                //                int ret = ZAux_Direct_Single_Cancel(handle, axis[i], 3);
                // commandCheckHandler("ZAux_Direct_Single_Cancel", ret);
            }
        }
    }
}
}  // namespace MoveControl
