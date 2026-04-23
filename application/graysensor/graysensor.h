//
// Created by LIMBO on 2026/4/21.
//

#ifndef INC_26IRCCSOFTWARE_GRAYSENSOR_H
#define INC_26IRCCSOFTWARE_GRAYSENSOR_H

/**
 * @brief 灰度传感器应用初始化,请在开启RTOS之前调用
 *
 */
void GraysensorInit();

/**
 * @brief 灰度传感器应用任务,放入实时系统以一定频率运行
 *
 */
void GraysensorTask();

#endif //INC_26IRCCSOFTWARE_GRAYSENSOR_H
