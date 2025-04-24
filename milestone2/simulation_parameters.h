#ifndef __SIMULATION_PARAMETERS_H__
#define __SIMULATION_PARAMETERS_H__

// Simulation creation is deterministic (1) or random (0)
extern int deterministic;

// Enables / disables collision
extern int enable_collision;

// Scenario. 1 = Moon; 2 = Mars; 3 = Earth
extern int scenario;

// Pump acceleration in m/s^2 at full throttle
extern float pump_acceleration;

// Pump delay in milliseconds every 10% of throttle change
extern int pump_delay;

// Drift acceleration in m/s^2 at full throttle
extern float drift_acceleration;

// Drift delay in milliseconds every 10% of throttle change
extern int drift_delay;

// Number of the GPS device read port
#define GPS_PORT_READ   60

// Data struct for gps data
typedef struct _gps_data
{
    float pod_x;            // Pod X position in landing zone (meters)
    float distance;         // Distance to platorm (meters)
} GPS_DATA, *PGPS_DATA;

// SENSORS device interrupt number
#define SENSORS_INTERRUPT_NUMBER 45

// Number of the SENSORS device read port
#define SENSORS_PORT_READ 21

// Data struct for sensors data
typedef struct _sensors_data
{
    float altitude;         // altitude of the pod (meters)
    float vertical_speed;   // vertical speed of the pod (meters/second)
    float distance_left;    // distance to the nearest object at the left (meters)
    float distance_right;   // distance to the nearest object at the right (meters)
} SENSORS_DATA, *PSENSORS_DATA;

// CLOCK device interrupt number
#define CLOCK_INTERRUPT_NUMBER 33

// PUMP device interrupt number
#define PUMP_INTERRUPT_NUMBER 14

// PUMP device write port
#define PUMP_PORT_WRITE 40

// DRIFT device interrupt number
#define DRIFT_INTERRUPT_NUMBER 15

// DRIFT device write port
#define DRIFT_PORT_WRITE 50

#endif
