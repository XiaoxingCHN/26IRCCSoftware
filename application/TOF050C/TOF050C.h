//
// Created by LIMBO on 2026/4/23.
//

#ifndef INC_26IRCCSOFTWARE_TOF050C_H
#define INC_26IRCCSOFTWARE_TOF050C_H

/**
 * @brief TOF050C应用初始化,请在开启RTOS之前调用
 *
 */
void TOF050CInit();

/**
 * @brief TOF050C应用任务,放入实时系统以一定频率运行
 *
 */
void TOF050CTask();

#endif //INC_26IRCCSOFTWARE_TOF050C_H
