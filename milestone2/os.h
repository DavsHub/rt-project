#ifndef __OS_H__
#define __OS_H__

#include <stdint.h>
#include <stddef.h>

#ifndef NULL
#define NULL (void*)0
#endif

#ifndef UNREFERENCED_PARAMETER
#define UNREFERENCED_PARAMETER(P) (P)=(P);
#endif

typedef struct _TASK TASK, *PTASK;

#define COREANY 0xFF

int TaskCreate(PTASK *task, char *name, int priority, int quantum, int core_affinity, void (*taskfunc)(void* param), void *param);

int TaskTerminate();

int TaskDelay(int milliseconds);

int TaskSuspend();

int TaskResume(PTASK task);

int TaskYield();

int TaskSetPreemption(int preemptable);

int TaskWaitForInterrupt(int InterruptNumber);

int *TaskGetErrnoAddress(PTASK task);

PTASK TaskCurrent();

#ifndef rtos_errno
#define rtos_errno (*TaskGetErrnoAddress(TaskCurrent()))
#endif

int ReadFromPort(int PortNumber, void *data, int bytes);

int WriteToPort(int PortNumber, void *data, int bytes);

uint64_t CpuGetCycles();

void EnableCompat();

void DisableCompat();

#endif
