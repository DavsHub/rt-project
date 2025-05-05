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

SEMAPHORE semAltStart, semAltDone, semDriftStart, semDriftDone, semGPSReady;
int milliseconds = 0;
SENSORS_DATA sensorData; 
GPS_DATA gpsData;
float XVel,XPos, targetX=500,d;
float targetAltitude=70, hoverThrust=-1.;
int alt_action, drift_action;

void SemaphoreInit(SEMAPHORE *sem, int initial) {
    sem->count = initial;
    sem->wait_count = 0;
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

void SemaphoreBroadcast(SEMAPHORE *sem) {
    TaskSetPreemption(0);
    for (int i = 0; i < sem->wait_count; ++i) {
        TaskResume(sem->waiting[i]);
    }
    sem->wait_count = 0;

    TaskSetPreemption(1);
}

void DetectHover() {
    const float P = 1, D=1.7, M = 0.2, tol=0.0000001;
    const int mc = 100;
    int c=0;
    float u = -1;
    hoverThrust = -1;
    TaskWaitForInterrupt (SENSORS_INTERRUPT_NUMBER);
    int targetAlt = sensorData.altitude;
    while (hoverThrust<0) {   
        TaskWaitForInterrupt (SENSORS_INTERRUPT_NUMBER); 
        float u_tmp = - (sensorData.altitude-targetAlt) * P \
        - (sensorData.vertical_speed) * D;
        u_tmp *= M;
        u_tmp = CLAMP(0, u_tmp, 1);
        //printf("u = %f\n", u_tmp);
        WriteToPort(PUMP_PORT_WRITE, &u_tmp, sizeof(float));
        if (fabs(u_tmp-u)<tol && u<1 && u >0) 
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
    const float P = 1, D=1.7, M = 0.2, tol=0.001;
    const float u_max = (2*hoverThrust<1)? 2*hoverThrust : 1;
    const float u_min = (2*hoverThrust<1)? 0 : 2*hoverThrust-1;
    float u=-1;

    while (1) {   
        TaskWaitForInterrupt (SENSORS_INTERRUPT_NUMBER); 
        u = - (sensorData.altitude-targetAltitude) * P \
            - sensorData.vertical_speed * D ;
        u *= M;
        u += hoverThrust;
        //refined max values to reduce overshoot
        u = sensorData.altitude-targetAltitude>0? CLAMP(u_min, u, 1):CLAMP(0, u, u_max);
        
        WriteToPort(PUMP_PORT_WRITE, &u, sizeof(float));
        //printf("%f %f %f\n",sensorData.altitude, sensorData.vertical_speed,u);
        if (fabs(u-hoverThrust)<tol) {
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

void flyToMinDist() {
    const float P =1, D=1.7, M=0.01, tol = 0.01;
    const int mc = 10;
    int c=0;
    float d = 1;

    WriteToPort(DRIFT_PORT_WRITE, &d, sizeof(float));
    TaskWaitForInterrupt (DRIFT_INTERRUPT_NUMBER);
    TaskDelay(2000);
    d = 0;
    WriteToPort(DRIFT_PORT_WRITE, &d, sizeof(float));
    GPS_DATA min_gpsData = gpsData;
    float old_pod_x = gpsData.pod_x;
    int old_milliseconds = milliseconds;
    while(1) {
        SemaphoreWait(&semGPSReady);
        //printf("%f %f\n",min_gpsData.distance, gpsData.distance);
        if (min_gpsData.distance>gpsData.distance) {
            min_gpsData = gpsData;
        } else {
            
            float delta = gpsData.pod_x-min_gpsData.pod_x;
            float deriv =  1000. * (gpsData.pod_x-old_pod_x)/(milliseconds-old_milliseconds);
            float d_tmp = 0;
            //printf("deriv: %f, delta: %f, time_delay= %d\n",deriv,gpsData.pod_x-old_pod_x, milliseconds-old_milliseconds);
            d_tmp = -P*delta- D*deriv;
            d_tmp *= M;
            d_tmp = CLAMP(-1,d_tmp,1);
            //printf("d: %f\n",d_tmp);

            
            if (d_tmp!=d) {
                d=d_tmp;
                WriteToPort(DRIFT_PORT_WRITE, &d, sizeof(float));
                TaskWaitForInterrupt(DRIFT_INTERRUPT_NUMBER);
            }
            if (fabs(d)<tol){   
                c++;
                if (c>=mc) {
                    SemaphoreSignal(&semDriftDone);
                    return;
                }
            } else {
                c=0;
            }
        }
        old_pod_x = gpsData.pod_x;
        old_milliseconds = milliseconds;
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
    int delay = (int) (intptr_t) param;
    while (1) {
        GPS_DATA gps_data_tmp;
        int res = ReadFromPort (GPS_PORT_READ , (void *) &gps_data_tmp ,sizeof(GPS_DATA));
        if (res == 0) {
            gpsData= gps_data_tmp;
            SemaphoreBroadcast(&semGPSReady);
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
        }        
    }
}

void Task_DriftCtrl(void *param) {
    UNREFERENCED_PARAMETER(param)

    while (1) {   
        SemaphoreWait(&semDriftStart);
        switch (drift_action) {
            case 0:
                boostForward(); // for testing
                break;
            case 1:
                flyToMinDist();
                break;
        }        
    }
}

void Task_Landing_Program(void *param) {
    UNREFERENCED_PARAMETER(param)
    TaskDelay(100);
    alt_action = 0;
    SemaphoreSignal(&semAltStart);
    SemaphoreWait(&semAltDone);
    while(sensorData.distance_right<gpsData.distance) {
        targetAltitude=sensorData.altitude+10;
        alt_action = 1;
        SemaphoreSignal(&semAltStart);
        SemaphoreWait(&semAltDone);
    }
    alt_action = 2;
    SemaphoreSignal(&semAltStart);
    SemaphoreWait(&semAltDone);
    drift_action=1;
    SemaphoreSignal(&semDriftStart);
    SemaphoreWait(&semDriftDone);
    targetAltitude=0;
    alt_action = 1;
    SemaphoreSignal(&semAltStart);
    SemaphoreWait(&semAltDone);
}


void InitTask(void *param)
{
    SemaphoreInit(&semAltDone,0);
    SemaphoreInit(&semAltStart,0);
    SemaphoreInit(&semDriftDone,0);
    SemaphoreInit(&semDriftStart,0);
    SemaphoreInit(&semGPSReady,0);
    PTASK task1,task2, task3, task4, task5, task6;
    TaskCreate(&task1, "Clock", 3, 1, 0, Task_Clock, NULL);
    TaskCreate(&task2, "Sensor", 2, 1, 0, Task_Sensor, NULL);
    TaskCreate(&task3, "GPS", 2, 1, 0, Task_GPS, (void *) 20);
    TaskCreate(&task4, "AltCtrl", 1, 1, 0, Task_AltitudeCtrl, NULL);
    TaskCreate(&task5, "DriftCtrl", 1, 1, 0, Task_DriftCtrl, NULL);
    TaskCreate(&task6, "Landing", 1, 1, 0, Task_Landing_Program, NULL);
}