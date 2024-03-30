## WHAT IS IT?

This air-to-air combat model provides a simple two-dimensional representation of red and blue aircraft flying within a wrapped 100 km^2 area (each patch represents 1000 meters). The model's main focus is to provide an implementation of proportional navigation guidance law in the air-to-air interceptor missiles carried by the blue aircraft. The law is simplified to operate in the 2D environment, but still follows the same principle. The default settings are representative of short range missiles guided by an infrared seeker, such as older versions of the AIM-9 Sidewinder. The model has been extended so that the blue aircraft also carry an onboard radar. The radar has very limited range and does not feed other systems. It is based on characteristics of the AN/APQ-153 used in the F-5.

## HOW IT WORKS

The simulation will start by creating a number of red and blue aircraft in random locations on the map. The number of aircraft is user defined. Each blue aircraft will be assigned a target, which is the closest red aircraft to it at the time of assignment. The blue aircraft will pursue its target, and when in range it will engage by firing a missile. If the missile misses the target and terminates, the blue aircraft will attempt to re-engage if within launch parameters. If the missile impacts the target and destroys it, the blue aircraft will be assigned the next closest red aircraft as its new target. This will continue until there are no red aircraft remaining.

The main purpose of this model is to explore the proportional navigation guidance law (https://en.wikipedia.org/wiki/Proportional_navigation) and the trajectory an interceptor missile will follow in order to impact its target. Proportional navigation's goal is to maintain the line-of-sight (LOS) rate from the missile to its target. In this model, the LOS rate is calculated by finding the difference between the missile's current relative bearing and the last recorded relative bearing in the previous timestep. The LOS rate is then fed into a PID controller as the process variable, with the setpoint being zero. The output value is used to control the intensity of the missile's maneuvering in order to maintain the LOS rate as close to zero as possible.

The onboard radar is modeled using a basic implementation of the radar range equation. Fixed parameters are entered based on the radar being modeled, such as transmitted power, antenna gain, bandwidth, and noise. Each aircraft is given a unique radar cross section (RCS) value. Aircraft within a pre-defined cone are considered to be in the radar's sensor volume, and calculations are performed with the equation to generate a signal to noise ratio (SNR) at each timestep. Future work will include a detection threshold, atmospheric noise, and an application of the radar's data to feed other targeting systems.

## HOW TO USE IT

Before running the simulation, a number of parameters can be set to vary aircraft speeds, the number of aircraft, and characteristics of the missiles. 

- patch-scale determines the factor used to scale other simulation values. This should not be changed.
- The units dropdown allows the user to enter either metric (m/s) or imperial (knots) values for the speed inputs.
- can-engage-AAM will prohibit blue from firing if set to OFF.
- The starting-red/starting-blue inputs allow the user to choose how many aircraft should be created.
- plane-speed and AAM-speed determine the constant speed of aircraft and missiles. Currently the AAM-speed value is not used.
- p-nav-gain, ki, and kd are parameters for tuning the PID controller on the missiles. These may cause undesired behavior if changed.

--- Other missiles parameters ---

- R-max is the maximum launch range for the interceptor missiles (km).
- max-tof is the maximum time of flight before missiles will terminate if they do not hit their target (sec).
- max-off-boresight-angle is the maximum angle away from an aircraft's nose that a missile can be launched. This is also used for radar azimuth scan limits (deg).
- max-AAM-turn-rate is that fastest possible rate that a missile can maneuver (deg/s).
- AAM-burn-time is how long the missile's engine will burn to produce thrust (sec).
- AAM-thrust is how much thrust the missile's engine will produce while it is burning (N).
- AAM-mass is the total mass of the missile (kg).

Once settings are configured as desired, the setup button should be pressed to initialize all the aircraft in the simulation. The go button should be pressed to begin the simulation. The inspect-aircraft input can be used to view data on a specific aircraft (ID). This is meant for blue aircraft. 

