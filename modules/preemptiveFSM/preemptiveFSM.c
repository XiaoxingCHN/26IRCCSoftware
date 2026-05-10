//
// Created by XIAOXINGCHN on 2026/5/8.
//
#include "preemptiveFSM.h"
#include "string.h"

static Pfsm_t *pfsm_table[PFSM_MAX_INSTANCES]; // 所有已经注册的PFSM指针数组
static uint8_t pfsm_count = 0;                 // 当前已经注册的PFSM实例数

static Pfsm_t *current_pfsm = NULL; // 当前正在运行的实例指针。NULL表示无

static uint32_t ready_bitmap =
    0; // 优先级位图，bit i为1表示由有限i的就绪PFSM实例

// 决策为单优先级决策
static Pfsm_t
    *ready_heads[PFSM_MAX_PRIORITY +
                 1]; // 就绪表头指针数组，每个元素指向对应优先级的就绪表头

// 取得最高优先级的就绪态任务
static Pfsm_t *PfsmSched_GetHighestReady(void);
/*
 * @brief 初始化PFSM调度器
 */
void PfsmSched_Init(void) {
  memset(pfsm_table, 0, sizeof(pfsm_table));
  memset(ready_heads, 0, sizeof(ready_heads));
  pfsm_count = 0;
  ready_bitmap = 0;
  current_pfsm = NULL;
}

void PfsmSched_DefaultRegister(Pfsm_t *pfsm, PfsmHandler handler,uint8_t priority) {
  if (pfsm_count >= PFSM_MAX_INSTANCES || priority > PFSM_MAX_PRIORITY) {
    return;
  }
  pfsm->handler = handler;
  pfsm->priority = priority;
  pfsm->state = PFSM_REDAY;
  pfsm->current_substate = 0;
  pfsm->pending_event = PFSM_EVENT_NONE;

  pfsm_table[pfsm_count] = pfsm;
  pfsm_count++;

  ready_bitmap |= (1 << priority);
  ready_heads[priority] = pfsm;
}

void PfsmSched_Register(Pfsm_t *pfsm, PfsmHandler handler, uint8_t priority) {
  if (pfsm_count >= PFSM_MAX_INSTANCES || priority > PFSM_MAX_PRIORITY) {
    return;
  }
  pfsm->handler = handler;
  pfsm->priority = priority;
  pfsm->state = PFSM_BLOCKED;
  pfsm->current_substate = 0;
  pfsm->pending_event = PFSM_EVENT_NONE;

  pfsm_table[pfsm_count] = pfsm;
  pfsm_count++;

  ready_heads[priority] = pfsm;
}

void PfsmSched_PostEvent(Pfsm_t *pfsm, PfsmEventId_e event) {
  if (pfsm == NULL||event == PFSM_EVENT_NONE)
    return;

  pfsm->pending_event = event;

  if (pfsm->state == PFSM_BLOCKED) {
    pfsm->state = PFSM_REDAY;
    ready_bitmap |= (1 << pfsm->priority);
    ready_heads[pfsm->priority] = pfsm; // 加入就绪表头
  }
}

void PfsmSched_Block(Pfsm_t *pfsm)
{
  if (current_pfsm != pfsm) return;

  pfsm->state = PFSM_BLOCKED;
  ready_bitmap &= ~(1 << pfsm->priority);
  if (ready_heads[pfsm->priority] == pfsm) {
    ready_heads[pfsm->priority] = NULL;
  }
  current_pfsm = NULL;
}

void PfsmSched_Run(void) {
    /* 无就绪 FSM，直接返回 */
    if (ready_bitmap == 0)
        return;

    /* 选出最高优先级就绪 FSM */
    Pfsm_t *next = PfsmSched_GetHighestReady();
    if (next == NULL)
        return;

    /* 上下文切换 */
    if (current_pfsm != next) {
        if (current_pfsm != NULL && current_pfsm->state == PFSM_RUNNING) {
            current_pfsm->state = PFSM_REDAY;
            ready_bitmap |= (1 << current_pfsm->priority);
            ready_heads[current_pfsm->priority] = current_pfsm;
        }
        next->state = PFSM_RUNNING;
        ready_bitmap &= ~(1 << next->priority);
        if (ready_heads[next->priority] == next)
            ready_heads[next->priority] = NULL;
        current_pfsm = next;
    }

    /* 总是调用 handler，由 handler 自己判断 event == NONE 时做不做 */
    PfsmEventId_e evt = current_pfsm->pending_event;
    current_pfsm->pending_event = PFSM_EVENT_NONE;
    current_pfsm->handler(current_pfsm, evt);

    /* handler 返回后，放回就绪（除非 handler 自己调了 Block） */
    if (current_pfsm->state == PFSM_RUNNING) {
        current_pfsm->state = PFSM_REDAY;
        ready_bitmap |= (1 << current_pfsm->priority);
        ready_heads[current_pfsm->priority] = current_pfsm;
    }
    current_pfsm = NULL;
}

static Pfsm_t *PfsmSched_GetHighestReady(void) {
  for (uint8_t prio = 0; prio <= PFSM_MAX_PRIORITY; prio++) {
    if (ready_bitmap & (1 << prio)) {
      Pfsm_t *f = ready_heads[prio];
      if (f != NULL && f->state == PFSM_REDAY) {
        return f;
      }
      for (int i = 0; i < pfsm_count; i++) {
        if (pfsm_table[i]->priority == prio &&
            pfsm_table[i]->state == PFSM_REDAY) {
          ready_heads[prio] = pfsm_table[i];
          return pfsm_table[i];
        }
      }
      ready_bitmap &= ~(1 << prio);
    }
  }
  return NULL;
}