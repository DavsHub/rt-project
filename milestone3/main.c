#include "os.h"
#include "simulation_parameters.h"
#include <stdint.h>
#include <stdio.h>
#include <math.h>

#define CLAMP01(x)  ((x) < 0.0f ? 0.0f : ((x) > 1.0f ? 1.0f : (x)))
#define CLAMP11(x)  ((x) < -1.0f ? -0.9f : ((x) > 1.0f ? .9f : (x)))
#define MAX_WAITING_TASKS 16

typedef struct {
    int count;
    PTASK waiting[MAX_WAITING_TASKS];
    int wait_count;
} SEMAPHORE;

SEMAPHORE semAltStart, semAltDone;
int milliseconds = 0;
SENSORS_DATA sensorData; 
GPS_DATA gpsData;
float XVel,XPos, targetX=500,d;
float targetAltitude=70, hoverThrust=-1., thrust = 0;
int alt_action;

void SemaphoreInit(SEMAPHORE *sem, int initial) {
    TaskSetPreemption(0);
    sem->count = initial;
    sem->wait_count = 0;
    TaskSetPreemption(1);
}

void SemaphoreWait(SEMAPHORE *sem) {
    TaskSetPreemption(0);
    if (sem->count > 0) {
        sem->count--;
    } else {
        if (sem->wait_count >= MAX_WAITING_TASKS) {
            // Optional: handle overflow (e.g., spin, panic, or error return)
            while (1) TaskYield(); // crude fallback
        }

        PTASK current = TaskCurrent();
        sem->waiting[sem->wait_count++] = current;
        TaskSuspend();  // current task suspends itself
    }
    TaskSetPreemption(1);
}

void SemaphoreSignal(SEMAPHORE *sem) {
    TaskSetPreemption(0);
    if (sem->wait_count > 0) {
        // Wake one waiting task
        PTASK task = sem->waiting[0];
        for (int i = 1; i < sem->wait_count; ++i) {
            sem->waiting[i - 1] = sem->waiting[i];
        }
        sem->wait_count--;
        TaskResume(task);
    } else {
        sem->count++;
    }
    TaskSetPreemption(1);
}


void DetectHover() {
    const float P = 0.2, D=2., M = 0.2, tolu=0.0000001;
    const int mc = 100;
    int c=0;
    float u = -1;
    hoverThrust = -1;
    while (hoverThrust<0) {   
        TaskWaitForInterrupt (SENSORS_INTERRUPT_NUMBER); 
        float u_tmp = - (sensorData.altitude-50) * P \
        - (sensorData.vertical_speed) * D;
        u_tmp *= M;
        u_tmp = CLAMP01(u_tmp);
        //printf("u = %f\n", u_tmp);
        WriteToPort(PUMP_PORT_WRITE, &u_tmp, sizeof(float));
        if (fabs(u_tmp-u)<tolu && u<1 && u >0) 
            c++;
        else 
            c = 0;
        if (c>mc) {
            hoverThrust = u_tmp;
            printf("hoverThrust = %f\n", hoverThrust);
            SemaphoreSignal(&semAltDone);
        }
        u = u_tmp;

    }
}

void flyToAltitude() {
    const float P = 0.2, D=2., M = 0.2, tolu=0.000001;
    float u=-1;
    while (1) {   
        TaskWaitForInterrupt (SENSORS_INTERRUPT_NUMBER); 
        u = - (sensorData.altitude-targetAltitude) * P \
            - sensorData.vertical_speed * D ;
        u *= M;
        u += hoverThrust;
        u = CLAMP01(u);
        WriteToPort(PUMP_PORT_WRITE, &u, sizeof(float));
        printf("%f %f %f\n",sensorData.altitude, sensorData.vertical_speed,u);
        if (fabs(u-hoverThrust)<tolu) {
            printf("Reached altitude = %f\n", targetAltitude);
            SemaphoreSignal(&semAltDone);
            return;
        }
    }
    
}

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
        //printf("altitude: %f\n", sensorData.altitude);
 
    }
}

void Task_GPS(void *param) {
    int freq = (int) (intptr_t) param;
    while (1) {
        GPS_DATA gps_data_tmp;
        int res = ReadFromPort (GPS_PORT_READ , (void *) &gps_data_tmp ,sizeof(GPS_DATA));
        if (res == 0) {
            gpsData= gps_data_tmp;
            
        }
        TaskDelay (freq);
        
    }
}

void Task_AltitudeCtrl(void *param) {
    UNREFERENCED_PARAMETER(param)
    const float P = 0.2, D=2., M = 0.2;
    float u=-1;
    
    while (1) {   
        SemaphoreWait(&semAltStart);
        switch (alt_action) {
            case 0:
                DetectHover();
                break;
            case 1:
                flyToAltitude();
                break;
            default:
                u = - (sensorData.altitude-targetAltitude) * P \
                - sensorData.vertical_speed * D ;
                u *= M;
                u += hoverThrust;
                u = CLAMP01(u);
                WriteToPort(PUMP_PORT_WRITE, &u, sizeof(float));
        }
            
    }
}

void Task_XCtrl(void *param) {
    UNREFERENCED_PARAMETER(param)
    const float K1 = 0.2, K2=1., M = 0.02;

    //printf("%d\n", res);
    float d=0;
    float dtemp;
    while (0)
    {   
        dtemp = -(XPos-targetX) * K1 -(XVel) * K2;
        dtemp *= M;
        dtemp = CLAMP11(dtemp);
        
        if (d != dtemp) {
            d = dtemp;
            int res = WriteToPort(DRIFT_PORT_WRITE, &d, sizeof(float));
            printf("res:%d, d: %f, posX: %f, VelX: %f\n", res,d, XPos, XVel);
            TaskWaitForInterrupt (DRIFT_INTERRUPT_NUMBER);
        }
        printf("%f xpos: %f xvel:%f\n",dtemp, XPos, XVel);
        TaskDelay(100);
    }
}

void Task_Landing_Program(void *param) {
    UNREFERENCED_PARAMETER(param)
    alt_action = 0;
    SemaphoreSignal(&semAltStart);
    SemaphoreWait(&semAltDone);
    targetAltitude=70;
    alt_action = 1;
    SemaphoreSignal(&semAltStart);
    SemaphoreWait(&semAltDone);
    float d = 0.8;
    WriteToPort(DRIFT_PORT_WRITE, &d, sizeof(float));
    TaskWaitForInterrupt (DRIFT_INTERRUPT_NUMBER);
    TaskDelay(300);
    d = 0;
    WriteToPort(DRIFT_PORT_WRITE, &d, sizeof(float));
}


void InitTask(void *param)
{
    SemaphoreInit(&semAltDone,0);
    SemaphoreInit(&semAltStart,0);
    PTASK task1,task2, task3, task4, task5, task6;
    TaskCreate(&task1, "Clock", 2, 1, 0, Task_Clock, NULL);
    TaskCreate(&task2, "Sensor", 2, 1, 0, Task_Sensor, NULL);
    TaskCreate(&task3, "GPS", 2, 1, 0, Task_GPS, (void *) 100);
    TaskCreate(&task4, "AltCtrl", 1, 1, 0, Task_AltitudeCtrl, NULL);
    TaskCreate(&task5, "XCtrl", 1, 1, 0, Task_XCtrl, NULL);
    TaskCreate(&task6, "Landing", 1, 1, 0, Task_Landing_Program, NULL);

    

}