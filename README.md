# Hybrid Electric Vehicle and Battery Electric Vehicle Simulation
## About
This projects models individual subsystems from battery electric vehicles as well as hybrid electric vehicles to perform drivecycle testing. 
The energy usage of each system is analyzed to compare the efficiency of each vehicle approach.  The hybrid vehicle model includes a series HEV
structure in which the small engine does not directly provide power to the wheels, but only charges the battery by spinning the genset motor.  
  
## Simulations
The drive cycle used for testing is the US06 drive cycle.  Simulations are run on each model for a single instance of the drive cycle
as well as a five drivecycles repeated cyclically.  The code can easily be extended to alternate drivecycles by adding the desired cycles
to the 'Drivecycles' block.

## Results
![alt text](https://github.com/cjbagwell/hev-bev-vehicle-simulation/blob/master/images/Battery_Electric_Vehicle_US06_Simulation.jpg)
![alt text](https://github.com/cjbagwell/hev-bev-vehicle-simulation/blob/master/images/Hybrid_Electric_Vehicle_US06_Simulation.jpg)
![alt text](https://github.com/cjbagwell/hev-bev-vehicle-simulation/blob/master/images/Battery_Electric_Vehicle_Cyclical_US06_Simulation.jpg)
![alt text](https://github.com/cjbagwell/hev-bev-vehicle-simulation/blob/master/images/Hybrid_Electric_Vehicle_Cyclical_US06_Simulation.jpg)
