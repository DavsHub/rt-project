#include "simulation_parameters.h"

// Simulation creation is deterministic (1) or random (0)
int deterministic = 1;

// Enables / disables collision
int enable_collision = 1;

// Scenario. 1 = Moon; 2 = Mars; 3 = Earth
int scenario = 3;

// Pump acceleration in m/s^2 at full throttle
float pump_acceleration = 20.0f;

// Pump delay in milliseconds every 10% of throttle change
int pump_delay = 40;

// Drift acceleration in m/s^2 at full throttle
float drift_acceleration = 10.0f;

// Drift delay in milliseconds every 10% of throttle change
int drift_delay = 40;
