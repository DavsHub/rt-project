#include "os.h"
#include "simulation_parameters.h"
#include <stdint.h>
#include <stdio.h>

#define CLAMP01(x)  ((x) < 0.0f ? 0.0f : ((x) > 1.0f ? 1.0f : (x)))
#define CLAMP11(x)  ((x) <= -1.0f ? -0.9f : ((x) >= 1.0f ? 0.9f : (x)))

int milliseconds = 0;
SENSORS_DATA sensorData; 
GPS_DATA gpsData;
float XVel3, targetX=10, targetXVel=0;
float targetAltitude=30, targetVSpeed=0, hoverThrust=0.13, thrust = 0;
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
    UNREFERENCED_PARAMETER(param)
    while (1)
    {
        TaskWaitForInterrupt (SENSORS_INTERRUPT_NUMBER) ; 
        ReadFromPort (SENSORS_PORT_READ , (void *) &sensorData ,sizeof(SENSORS_DATA));
 
    }
}

void Task_GPS(void *param) {
    int freq = (int) (intptr_t) param;
    int prev_time = -1;
    while (1)
    {
        GPS_DATA gps_data_tmp;
        int res = ReadFromPort (GPS_PORT_READ , (void *) &gps_data_tmp ,sizeof(GPS_DATA));
        if (res == 0) {
            if (prev_time != -1) {
                XVel3 = (gps_data_tmp.pod_x-gpsData.pod_x)/(milliseconds-prev_time);
            }
            printf("%f %f\n", XVel3, gpsData.pod_x);
            prev_time = milliseconds;
            gpsData= gps_data_tmp;
        }

        TaskDelay (freq);
    }
}

void Task_AltitudeCtrl(void *param) {
    UNREFERENCED_PARAMETER(param)
    const float K1 = 0.5, K2=2., M = 0.2;
    while (1)
    {   
        TaskWaitForInterrupt (SENSORS_INTERRUPT_NUMBER) ; 
        float u = - (sensorData.altitude-targetAltitude) * K1 \
                  - (sensorData.vertical_speed-targetVSpeed) * K2;
        u *= M;
        u += hoverThrust;
        u = CLAMP01(u);
        WriteToPort(PUMP_PORT_WRITE, &u, sizeof(float));
    }
}

void Task_XCtrl(void *param) {
    UNREFERENCED_PARAMETER(param)
    const float K1 = 0.5, M = 0.2;
    float d,dtemp;
    while (1)
    {   
        dtemp = -(gpsData.pod_x-targetX) * K1;
                   //+(XVel3-targetXVel) * K2;
        dtemp *= M;
        dtemp = CLAMP11(dtemp);
        d=dtemp;
        int res= WriteToPort(DRIFT_PORT_WRITE, &d, sizeof(float));
        printf("%d %f\n", res, d);
        TaskDelay (1000);
    }
}

void InitTask(void *param)
{
    PTASK task1,task2, task3, task4, task5;
    TaskCreate(&task1, "Clock", 2, 1, 0, Task_Clock, NULL);
    TaskCreate(&task2, "Sensor", 2, 1, 0, Task_Sensor, NULL);
    TaskCreate(&task3, "GPS", 2, 1, 0, Task_GPS, (void *) 100);
    TaskCreate(&task4, "AltCtrl", 1, 1, 0, Task_AltitudeCtrl, NULL);
    TaskCreate(&task5, "XCtrl", 1, 1, 0, Task_XCtrl, NULL);
}