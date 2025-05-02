#include "os.h"
#include "simulation_parameters.h"
#include <stdint.h>
#include <stdio.h>
#include <math.h>

#define CLAMP(x,y,z)  ((y) < x ? x : ((y) > z ? z : (y)))

#define MAX_WAITING_TASKS 16

typedef struct {
    int count;
    PTASK waiting[MAX_WAITING_TASKS];
    int wait_count;
} SEMAPHORE;

SEMAPHORE semAltStart, semAltDone, semDriftStart, semDriftDone;
int milliseconds = 0;
SENSORS_DATA sensorData; 
GPS_DATA gpsData;
float XVel,XPos, targetX=500,d;
float targetAltitude=70, hoverThrust=-1.;
int alt_action, drift_action;

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
    const float P = 1, D=1.7, M = 0.2, tolu=0.0000001;
    const int mc = 100;
    int c=0;
    float u = -1;
    hoverThrust = -1;
    while (hoverThrust<0) {   
        TaskWaitForInterrupt (SENSORS_INTERRUPT_NUMBER); 
        float u_tmp = - (sensorData.altitude-50) * P \
        - (sensorData.vertical_speed) * D;
        u_tmp *= M;
        u_tmp = CLAMP(0, u_tmp, 1);
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
    const float P = 1, D=1.7, M = 0.2, tolu=0.000001;
    float u=-1;
    while (1) {   
        TaskWaitForInterrupt (SENSORS_INTERRUPT_NUMBER); 
        u = - (sensorData.altitude-targetAltitude) * P \
            - sensorData.vertical_speed * D ;
        u *= M;
        u += hoverThrust;
        u = CLAMP(0, u, 2*hoverThrust);
        
        WriteToPort(PUMP_PORT_WRITE, &u, sizeof(float));
        //printf("%f %f %f\n",sensorData.altitude, sensorData.vertical_speed,u);
        if (fabs(u-hoverThrust)<tolu) {
            printf("Reached altitude = %f\n", targetAltitude);
            SemaphoreSignal(&semAltDone);
            return;
        }
    }
    
}

void hover() {
    const float D=1.7, M = 0.2, tol=0.000001;
    const int mc = 100;
    int c=0;
    float u;
    while (1) {   
        TaskWaitForInterrupt (SENSORS_INTERRUPT_NUMBER); 
        u = - sensorData.vertical_speed * D ;
        u *= M;
        u += hoverThrust;
        u = CLAMP(0, u, 2*hoverThrust);
        WriteToPort(PUMP_PORT_WRITE, &u, sizeof(float));
        //printf("%f %f\n",sensorData.vertical_speed,u);
        if (fabs(sensorData.vertical_speed)<tol) {
            c++;
        } else {
            c=0;
        }
        if (c>mc) {
            printf("hovering\n");
            SemaphoreSignal(&semAltDone);
            return;
        }
    }
    
}

void boostForward() {
    float d = 1.0;
    WriteToPort(DRIFT_PORT_WRITE, &d, sizeof(float));
    TaskWaitForInterrupt (DRIFT_INTERRUPT_NUMBER);
    TaskDelay(1000);
    d = 0.;
    WriteToPort(DRIFT_PORT_WRITE, &d, sizeof(float));
    SemaphoreSignal(&semDriftDone);
    return;
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
    int delay = (int) (intptr_t) param;
    while (1) {
        GPS_DATA gps_data_tmp;
        int res = ReadFromPort (GPS_PORT_READ , (void *) &gps_data_tmp ,sizeof(GPS_DATA));
        if (res == 0) {
            gpsData= gps_data_tmp;
            
        }
        TaskDelay (delay);
        
    }
}

void Task_AltitudeCtrl(void *param) {
    UNREFERENCED_PARAMETER(param) 
    while (1) {   
        SemaphoreWait(&semAltStart);
        switch (alt_action) {
            case 0:
                DetectHover();
                break;
            case 1:
                flyToAltitude();
                break;
            case 2:
                hover();
                break;
            default:
                hover();
        }        
    }
}

void Task_DriftCtrl(void *param) {
    UNREFERENCED_PARAMETER(param)

    while (1) {   
        SemaphoreWait(&semDriftStart);
        switch (drift_action) {
            case 0:
                boostForward();
                break;
            case 1:
                //flyToMinDist();
                break;
            default:
                hover();
        }        
    }
    const float K1 = 0.2, K2=1., M = 0.02;

    //printf("%d\n", res);
    float d=0;
    float dtemp;
    while (0)
    {   
        dtemp = -(XPos-targetX) * K1 -(XVel) * K2;
        dtemp *= M;
        dtemp = CLAMP(-1,dtemp,1);
        
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
    alt_action = 2;
    SemaphoreSignal(&semAltStart);
    SemaphoreWait(&semAltDone);
    drift_action=0;
    SemaphoreSignal(&semDriftStart);
    SemaphoreWait(&semDriftDone);
    while(1){
        TaskDelay(100);
        printf("xpos: %f distance:%f\n", gpsData.pod_x, gpsData.distance);
    }
}


void InitTask(void *param)
{
    SemaphoreInit(&semAltDone,0);
    SemaphoreInit(&semAltStart,0);
    SemaphoreInit(&semDriftDone,0);
    SemaphoreInit(&semDriftStart,0);
    PTASK task1,task2, task3, task4, task5, task6;
    TaskCreate(&task1, "Clock", 3, 1, 0, Task_Clock, NULL);
    TaskCreate(&task2, "Sensor", 2, 1, 0, Task_Sensor, NULL);
    TaskCreate(&task3, "GPS", 2, 1, 0, Task_GPS, (void *) 20);
    TaskCreate(&task4, "AltCtrl", 1, 1, 0, Task_AltitudeCtrl, NULL);
    TaskCreate(&task5, "DriftCtrl", 1, 1, 0, Task_DriftCtrl, NULL);
    TaskCreate(&task6, "Landing", 1, 1, 0, Task_Landing_Program, NULL);
}