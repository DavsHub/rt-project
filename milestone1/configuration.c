#include "configuration.h"

// Number of cores
int CPU_CORES = 1;

// CPU frequency (in KHz)
int CPU_FREQUENCY = 100;

// Interval between 2 clock interrupts (in milliseconds)
int CLOCK_INTERRUPT_INTERVAL = 1;

// Priority of the INIT task
int INIT_PRIORITY = 1;

// Priority of the IDLE task
int IDLE_PRIORITY = 0;

// Topology of the ready_queue (global o per core)
int READY_QUEUE = RQ_GLOBAL;

// Number of instructions per simulated cycle
int INSTRUCTIONS_PER_CYCLE = 1;
