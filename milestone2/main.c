#include "os.h"
#include "simulation_parameters.h"
#include <stdint.h>
#include <stdio.h>

int milliseconds = 0;
SENSORS_DATA sensord; 
GPS_DATA gpsData;

void Task_Clock ( void * param )
{
    UNREFERENCED_PARAMETER ( param )

    while (1)
    {
        TaskWaitForInterrupt (CLOCK_INTERRUPT_NUMBER) ; 
        milliseconds ++;
    }
}

void Task_Sensor(void *param) {
    UNREFERENCED_PARAMETER ( param )
    while (1)
    {
        TaskWaitForInterrupt (SENSORS_INTERRUPT_NUMBER) ; 
        ReadFromPort (SENSORS_PORT_READ , (void *) &sensord ,sizeof(SENSORS_DATA));
        if (sensord.altitude>0) printf("%d: %f %f %f\n", milliseconds, sensord.altitude, gpsData.distance, gpsData.pod_x);
    }
}

void Task_GPS(void *param) {
    int freq = (int) (intptr_t) param;
    while (1)
    {
        GPS_DATA gps_data_tmp;
        int res = ReadFromPort (GPS_PORT_READ , (void *) &gps_data_tmp ,sizeof(GPS_DATA));
        if (res == 0) gpsData= gps_data_tmp;
        TaskDelay (freq);
    }
}


void InitTask(void *param)
{
    PTASK task1,task2, task3;
    TaskCreate(&task1, "Clock", 1, 1, 0, Task_Clock, NULL);
    TaskCreate(&task2, "Sensor", 1, 1, 0, Task_Sensor, NULL);
    TaskCreate(&task3, "GPS", 1, 1, 0, Task_GPS, (void *) 100);
}