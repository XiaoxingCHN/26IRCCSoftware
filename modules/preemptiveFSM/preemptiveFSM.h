//
// Created by XIAOXINGCHN on 2026/5/8.
//

#ifndef INC_26IRCCSOFTWARE_PREEMPTIVEFSM_H
#define INC_26IRCCSOFTWARE_PREEMPTIVEFSM_H

#include "main.h"
#include "stdbool.h"
#include "stdint.h"

/*配置参数*/
#define PFSM_MAX_INSTANCES 10 // 最多支持PFSM的实例数
#define PFSM_MAX_PRIORITY 10  // 最大优先级,0-10,0级为最高优先级

// PFSM的运行状态
typedef enum {
  PFSM_REDAY = 0, // 就绪状，已登记在就绪表中
  PFSM_RUNNING, // 运行中，正在执行当前状态拥有调度器的执行权
  PFSM_BLOCKED, // 阻塞中，等待外部事件触发或超时，不在就序列表中
} Pfsmstate_e;

// PFSM的事件类型
typedef enum {
  PFSM_EVENT_NONE = 0, // 无事件
  PFSM_EVENT_TIMER,    // 超时事件
  /*Begin 自定义事件*/
  PFSM_EVENT_FINISH_LOADPLATFORM,//完成登台
  PFSM_EVENT_FALLDOWN_PLATFORM, // 掉台事件
  PFSM_EVENT_FINDOUT_AIM,//发现目标事件9
  /*End 自定义事件*/
  PFSM_EVENT_COUNT, // 计数事件,勿要使用
} PfsmEventId_e;

// 前向声明
typedef struct Pfsm Pfsm_t;

// PFSM处理函数
typedef void (*PfsmHandler)(Pfsm_t *pfsm, PfsmEventId_e event);

// Pfsm控制块
struct Pfsm {
  PfsmHandler handler;      // 自定义的状态机函数
  uint8_t priority;         // 优先级,0-10,0级为最高优先级
  Pfsmstate_e state;        // 当前状态
  uint8_t current_substate; // 当前的子状态仅限内部使用

  PfsmEventId_e pending_event; // 待处理事件
};

// 对外的API接口
/*
 *  @brief 初始化PFSM调度器
 */
void PfsmSched_Init(void);
/*
 *  @brief 注册一个PFSM实例，默认任务绑定处理函数和优先级，注册后立即处于就绪态
 *  @param pfsm PFSM实例指针
 *  @param handler PFSM处理函数指针
 *  @param priority PFSM优先级,0-10,0级为最高优先级
 */
void PfsmSched_DefaultRegister(Pfsm_t *pfsm, PfsmHandler handler,uint8_t priority);
/*
 *  @brief 注册一个PFSM实例，绑定处理函数和优先级，注册后立即处于阻塞态
 *  @param pfsm PFSM实例指针
 *  @param handler PFSM处理函数指针
 *  @param priority PFSM优先级,0-10,0级为最高优先级
 */
void PfsmSched_Register(Pfsm_t *pfsm, PfsmHandler handler, uint8_t priority);

/*Foidsmchfoisdjfoidsjfiojsdf
 *  @brief 向指定FSM投递事件
 *  @param pfsm PFSM实例指针
 *  @param event PFSM事件类型
 */
void PfsmSched_PostEvent(Pfsm_t *pfsm, PfsmEventId_e event);
/*
 *  @brief 阻塞指定FSM实例，等待外部事件触发或超时
 *  @param pfsm PFSM实例指针
 */
void PfsmSched_Block(Pfsm_t *pfsm);
// 调度器主循环在需要的主任务中反复调用
/*
 *  @brief 调度器主循环，需要在主任务中反复调用
 */
void PfsmSched_Run(void);

#endif // INC_26IRCCSOFTWARE_PREEMPTIVEFSM_H
