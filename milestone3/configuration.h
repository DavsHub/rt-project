#ifndef __CONFIGURATION_H__
#define __CONFIGURATION_H__

// Number of cores
extern int CPU_CORES;

// CPU frequency (in KHz)
extern int CPU_FREQUENCY;

// Interval between 2 clock interrupts (in milliseconds)
extern int CLOCK_INTERRUPT_INTERVAL;

// Priority of the INIT task
extern int INIT_PRIORITY;

// Priority of the IDLE task
extern int IDLE_PRIORITY;

#define RQ_GLOBAL  0
#define RQ_PERCORE 1

// Topology of the ready_queue (global o per core)
extern int READY_QUEUE;

// Number of instructions per simulated cycle
extern int INSTRUCTIONS_PER_CYCLE;

#endif
