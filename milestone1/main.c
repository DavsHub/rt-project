#include "os.h"
#include "simulation_parameters.h"
void TaskFloat( void *param) {
    float th = 0.13;
    WriteToPort(PUMP_PORT_WRITE, &th, sizeof(float));
}

void InitTask(void *param)
{
    PTASK task;
    TaskCreate(&task, "Float", 1, 1, 0, TaskFloat, NULL);
}