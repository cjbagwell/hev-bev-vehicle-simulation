% ME 591-009 Project 4 Matlab Script File
% Create By: C. Jordan Bagwell
% Date: 3/28/2020
% Description: The code populates values into the associated Simulink
% model.  

%% Prepare Workspace 
close all;  clear;  clc;

% Set file variables
BEV_SIMULATION_NAME = 'BEVModel';
HEV_SIMULATION_NAME = 'HEVModel';
DC_GENSET_SIMULATION_NAME = 'DCMotorGenset';
GRADUATE_SIMULATION_NAME = 'HEV_DCModel';
GRADUATE_SIMULATION_NAME_2 = 'HEV_DCModel_NewBatteryPack';
GENERATOR_MOTOR_DATA_NAME = 'GeneratorMotorData.mat';

genMotorData = load(GENERATOR_MOTOR_DATA_NAME);

% Variable Definitions

% Vehicle Variables
m = 1965;       %Vehicle Mass [kg]
Cd = 0.576;     %Drag Coefficient [-]
Af = 1.00;      %Frontal Area [m^2]
Crr = 0.01;     %Rolling Coefficient [-]
r = 0.290;      %Effective Tire Radius [m]
brakeGain= 215; %Brake Pedal Position Gain
theta = 0;      %angle of incline [rad]

% Physics Variables
rho = 1.20;     %Air Density [kg/m^3]
g = 9.81;       %Gravitational Acceleration [m/s^2]
dt = 0.01;      %Time step [s]
tBrake2 = 8.57; %WOT Cycle End Time [s]
vRef = 60;
tEnd = 10;
cycleNum = 1;

% Controller ("Driver") Parameters
Kp = 200;       %Proportional Gain
Ki = 0.1;       %Integral Gain
Kd = 0;         %DerivativeGain

% Motor Parameters
kc = 0.513;     %
ki = 0.0773;    %
kw = 0.0000064; %
kt = 3;         %Torque Constant [Nm/A]
ke = kt;        %Back EMF Constant [Nm/A]
Ra = .06;       %Winding Resistance [ohm]
Jm = 5;         %Rotational Inertia [kg/m^2]
La = .01;       %Motor Inductance [H]
b = .002;       %Rotational damping
C = 400;        % [W]
motorMaxTorque = 600;       %Motor Max Torque [N-m]
motorMaxPower = 310000;     %Motor Max Power [W]
motorMaxSpeed = 18000;      %Motor Max Electrical Speed [rpm]

% Genset Parameters
etaGenGR = 0.93;            %Genset Gearbox Efficiency
GRgen = 1.3;                %Genset Gearbox Gear Ratio
initialGasVolume = 5;       %Initial Volume of Gasoline in Tank [gal]

% Grad Portion Parameters
KP_DC = 200;
KI_DC = 0.1;
KD_DC = 0;

% Brake Parameters
meanPadRadius = .20;        %Mean Pad Radius of Brakes [m]
cylinderBore = .01;         %Brake Cylinder Bore [m]
numPads = 2;                %Number of Brake Pads [~]
mu_k = 0.7;                 %Coefficient of Kinetic Friction [~]
mu_s = 0.9;                 %Coefficient of Static Friction [~]
mu_v = 0;                   %Coefficient of Viscous Friction [~]
FbrakeMax = 12000;          %Max Braking Force [N]
omegaBreakAway = 0.01;      %Break Away Velocity (static to kinetic) [rad/s]

% Battery Parameters
batteryResistance = 0.0461; %Battery Internal Resistance [ohm]
openCircuitVoltage = 420;   %Battery Open Circuit Voltage [V]
batteryCapacity = 75;       %Battery Capacity [kWh]
SOCi = 95;                  %Battery Initial State Of Charge [%]
accessoryLoad = 600;        %[W]

% Driveline Variables
GRd = 9.843;     %Differential Gear Ratio
GEd = 1;        %differential Gear Efficiency [%]
EffTableSpeed = [0 5 10 15 25 35 45 55 65];
EffTableEff = [.75 .85 .89 .95 .96 .97 .98 .99 .99];

% Initial Conditions
v0 = 0;         %initial velocity [m/s]
x0 = 0;         %initial position [m]
APP0 = 0;       %initial accelerator pedal position [%]
tEndWOT2 = 50;  %simulation end time of WOT (to populate model)

% Drive Cycle Parameters
tEndUS06 = 600;     tEndUDDS = 1369;    
tEndHwFET = 765;    tEndFTP75 = 2474;  
tEndWOT = 15;       tBrake = 7;
tEndWOT2 = 15;      tBrake2 = 9;

% Unit Conversion Factors
k_ftlbs2Nm = 1.3448;        %[Nm/ft-lb]
k_mph2mps = 0.44704;        %[mps/mph]
k_mps2mph = 1/k_mph2mps;    %[mph/mps]
k_gal2L = 3.78541;          %[L/gal]
k_N2kN = 1/1000;            %[kN/N]
k_W2kW = 1/1000;            %[kW/W]
k_m2km = 1/1000;            %[km/m]
k_kW2W = 1000;              %[W/kW]
k_s2h = 1/3600;             %[h/s]
k_kW2hp = 1.34102;          %[hp/kW]
k_m2mi = 0.000621371;       %[mi/m]
k_kWh2gal = 1/32.3;         %[gal/kWh]
k_min2s = 60;               %[s/min]
k_rps2rpm = 60/(2*pi);      %[s-rev/rad-min]
k_kWh2Ws = 3600*1000;       %[Ws/kWh]
k_Ws2gal = 1/3600/1000/32.3;%[gal/Ws]
k_hr2s = 3600;              %[s/hr]
k_kph2mps = 1000/3600;      %[m-hr/km-s]
k_J2kWh = 1/3600000;        %[kWh/J]
k_J2gal = 1/3600000/32.3;   %[gal/J]
k_rpm2rps = 2*pi()/60;      %[rad-min/s-rev]
k_m32gal = 264.172;         %[gal/m^3]
k_gal2kWh = 32.3;           %[kWh/gal]
k_kWh2J = 3.6*10^6;         %[J/kWh]


%% Section 1: Vehicle Concept Questions (Based on BEV Model)
clc;
fprintf('Section 1: Vehicle Concept Questions\n\n')

% Question 1.4
Pout = 457;
etaEngine = 0.35;
Pin = Pout / etaEngine;
Ploss = Pin - Pout;
percentQ = 0.5;
radiatiorRaditing = Ploss * percentQ;
out = append('Question 1.4:',...
    '\tWhen designing a thermal system to control the temperature in an internal\n',...
    '\tcombustion engine, what should theapproximate rating of the radiator be if\n',...
    '\tthe engine is expected to produce 457 hp at the crankshaft?\n', ...
    '\n',...
    '\tThe approximate power rating of the radiator is %.0f hp.\n\n');
fprintf(out, radiatiorRaditing);

% Question 1.5
Pw = motorMaxPower * 0.99;
a = 1/2*rho*Af*Cd;
bPly = 0;
c = Crr*m*g;
d= -Pw;
polyCoeff = [a bPly c d];
vMax = roots(polyCoeff)*k_mps2mph;
out = append('Question 1.5:',...
    '\tCalculate the drag limited top speed of the vehicle assuming there are no \n',...
    '\tlimits on component speeds.\n\n',...
    '\tThe limited top speed is %.0f mph\n\n');
fprintf(out, vMax(3));

% Question 1.6
v = 105 * k_kph2mps;
deltaT = 3 * k_hr2s;
Edrag = (1/2*rho*Af*Cd*v^2 + Crr*m*g*cos(theta)) * v*deltaT /1000;
out = append('Question 1.6:',...
    '\tHow much energy is consumed from drag forces when the vehicle travels at 105 kph\n',...
    '\tfor 3 hours in kJ?\n\n',...
    '\tThe energy comsumed from drag forces is approximately %.0f kj.\n\n');
fprintf(out, round(Edrag, -3));

% Question 1.7
Ftr = 1/2*rho*Cd*Af*v^2 + Crr*m*g*cos(theta);
APP7 = 100*(Ftr*r)/(motorMaxTorque * 0.965);
deltaX = v*k_mps2mph*3;
MPGe = deltaX*.965*.93/(Edrag*k_J2kWh*k_kWh2gal) / 1000;
out = append('Question 1.7:',...
    '\tWhat is the APP value at a steady state of 105 kph? What is the equivalent fuel\n',...
    'economy in MPGe, assuming 93%% efficiency for battery pack losses?\n\n',...
    '\tThe APP is approximately %.1f%% and the fuel economy equivelent is %.0f.\n\n');
fprintf(out, APP7, MPGe);


%% Section 2: Vehicle Modeling 
fprintf('Section 2: Vehicle Modeling \n')
fprintf('In this section, simulations are run on vehicle models created ')
fprintf('in Simulink \n')


%% Section 2.1: Brake System Modeling
% Question 2.1.1
TbrakeMax = FbrakeMax*r;     %Max Braking Torque [N-m]
pBrakeMax = 4*TbrakeMax / (mu_k*pi()* cylinderBore^2 *meanPadRadius*numPads);

% Question 2.1.3
omega = 100;
Pdissipate = TbrakeMax * omega * k_W2kW;

% Plot Drivecyle US06
cycleNum = 2;   
SOCi = 90
tEnd = tEndUS06;
fprintf("Running Battery Electric Vehicle Simulation on US06 Drivecycle... ")
sec21Results = sim(BEV_SIMULATION_NAME);
fprintf("Finished!\n")
figure();   
subplot(2, 2, 1); hold on;
sgtitle("Battery Electric Vehicle US06 Simulation")
sec21Results.driver.velocitySetpoint.plot
sec21Results.vehicleDynamics.velocity.plot
title('Velocity');   xlabel('Time [s]'); ylabel('Velocity [m/s]')
legend('Setpoint','Actual')

% Plot MPGe
hold off
subplot(2, 2, 2)
sec21Results.battery.MPGe.plot
title('Fuel Economy');   xlabel('Time [s]'); ylabel('MPGe [mpg]')

% Plot SOC
subplot(2, 2, 3)
sec21Results.battery.SOC.plot
title('SOC');       xlabel('Time [s]'); ylabel('SOC [%]')

% Plot Cyclical Drivecyle US06
cycleNum = 2;   
tEnd = tEndUS06 * 5;
SOCi = 90
fprintf("Running Battery Electric Vehicle Simulation on US06 Drivecycle... ")
cycBevResults = sim(BEV_SIMULATION_NAME);
fprintf("Finished!\n")
figure();   
subplot(2, 2, 1); hold on;
sgtitle("Battery Electric Vehicle Cyclical US06 Simulation")
cycBevResults.driver.velocitySetpoint.plot
cycBevResults.vehicleDynamics.velocity.plot
title('Velocity');   xlabel('Time [s]'); ylabel('Velocity [m/s]')
legend('Setpiont','Actual')

% Plot MPGe
hold off
subplot(1, 2, 2)
cycBevResults.battery.MPGe.plot
title('Fuel Economy');   xlabel('Time [s]'); ylabel('MPGe [mpg]')

% Plot SOC
subplot(2, 2, 3)
cycBevResults.battery.SOC.plot
title('SOC');       xlabel('Time [s]'); ylabel('SOC [%]')


%% Section 2.2: Modeling a Generator Set
% implement new hybrid configuration
batteryCapacity = 50;       %[kWh]

% Question 2.2.1
GRequired = 25*k_gal2kWh;

% Question 2.2.5
GRgen = 1.4;


%% Section 2.3: Series Hyrbrid Model
SOCi = 90;  tEnd = tEndUS06;
fprintf("Running the Hybrid Electric Model on US06 Drivecycle... ")
sec23Results = sim(HEV_SIMULATION_NAME);
fprintf("Finished!\n")
results = sec23Results;

% Question 2.3.1: plot drivecylce and actual velocity
figure();
sgtitle("Hybrid Electric US06 Simulation")
subplot(2, 3, 1); hold on;
results.driver.velocitySetpoint.plot
results.vehicleDynamics.velocity.plot
legend('Velocity Setpoint [m/s]','Actual Velocity [m/s]')
title('Velocity')
% Question 2.3.2: Plot SOC
subplot(2, 3, 2);
results.battery.SOC.plot
ylabel('%')
title('SOC')
% Question 2.3.3: Plot fuel left in gas tank
subplot(2, 3, 3);
results.genset.engine.gasTankVolume.plot
title('Fuel Volume')
ylabel('gal')
% Question 2.3.4: Plot actual Engine Torque and Actual Motor Speed
subplot(2, 3, 4);     hold on
results.genset.engine.brakeTorque.plot
results.genset.motor.rpm.plot
legend('Engine Brake Torque [Nm]','Motor Angular Velocity [rpm]')
title('Genset Data')
% Question 2.3.5: Plot MPGe
subplot(2, 3, 5);
results.battery.MPGe.plot
ylabel('mpg')
title('Fuel Economy - MPGe')
% Question 2.3.6


% Run Cyclical Drivecycle Simulation
SOCi = 90;  tEnd = tEndUS06 * 5;
fprintf("Running the Hybrid Electric Model on US06 Drivecycle... ")
sec23Results = sim(HEV_SIMULATION_NAME);
fprintf("Finished!\n")
results = sec23Results;

% Question 2.3.1: plot drivecylce and actual velocity
figure();
sgtitle("Hybrid Electric Cyclical US06 Simulation")
subplot(2, 3, 1); hold on;
results.driver.velocitySetpoint.plot
results.vehicleDynamics.velocity.plot
legend('Setpoint','Actual')
title('Velocity')
% Question 2.3.2: Plot SOC
subplot(2, 3, 2);
results.battery.SOC.plot
ylabel('%')
title('SOC')
% Question 2.3.3: Plot fuel left in gas tank
subplot(2, 3, 3);
results.genset.engine.gasTankVolume.plot
title('Fuel Volume')
ylabel('gal')
% Question 2.3.4: Plot actual Engine Torque and Actual Motor Speed
subplot(2, 3, 4);     hold on
results.genset.engine.brakeTorque.plot
results.genset.motor.rpm.plot
legend('Engine Brake Torque [Nm]','Motor Angular Velocity [rpm]')
title('Genset Data')
% Question 2.3.5: Plot MPGe
subplot(2, 3, 5);
results.battery.MPGe.plot
ylabel('mpg')
title('Fuel Economy - MPGe')

%% Section 3.1: Open-Loop DC Motor Genset
GRgen = 1.3;
torqueCommand = 65;     %[Nm]
motorSpeedCommand = 800;%[rpm]
fprintf("Running the DC Motor Genset model with setpoints 65 Nm at 800 RPM... ");
sec31Results = sim(DC_GENSET_SIMULATION_NAME);  results = sec31Results;
fprintf("Finished!\n");

% Question 3.1.1
figure; hold on
results.genset.engine.powerIn.plot
results.genset.motor.idealPower.plot
xlabel('Time [s]'); ylabel('Power [kW]')
legend('Engine Input Power','Ideal Electrical Power')
title('DC Motor Genset Powers')
% Question 3.1.2
figure; hold on
results.genset.gearbox.powerLoss.plot
results.genset.motor.dampingPowerLoss.plot
results.genset.motor.resistancePowerLoss.plot
xlabel('Time [s]'); ylabel('Power [kW]')
legend('Gearbox Power Loss','Damping Power Loss','Resistance Power Loss')
title('DC Motor Genset Power Losses')
% Question 3.1.3

% Question 3.1.4


%% Section 3.2: DC Motor Genset Implementation Into Series Hybrid
tEnd = tEndUS06*5;  cycleNum = 2;   GRgen = 1.4;
fprintf("Running DC Motor Genset in Series Hybrid simulation on US06 Drivecycle... ")
sec32Results = sim(GRADUATE_SIMULATION_NAME);  results = sec32Results;
fprintf("Finished!\n")
% Question 3.2.1: plot drivecylce and actual velocity
figure();   hold on
results.driver.velocitySetpoint.plot
results.vehicleDynamics.velocity.plot
legend('Velocity Setpoint [m/s]','Actual Velocity [m/s]')
title('Cyclical US06 Drivecycle Velocity')
% Question 3.2.2: Plot SOC
figure;
results.battery.SOC.plot
ylabel('%')
title('Cyclical US06 SOC')
% Question 3.2.3: Plot fuel left in gas tank
figure;
results.genset.engine.gasTankVolume.plot
title('Cyclical US06 Genset Fuel Volume')
ylabel('gal')
% Question 3.2.4: Plot actual Engine Torque and Actual Motor Speed
figure;     hold on
results.genset.engine.brakeTorque.plot
results.genset.motor.rpm.plot
legend('Engine Brake Torque [Nm]','Motor Angular Velocity [rpm]')
title('Cyclical US06 Genset Data')
% Question 3.2.5: Plot MPGe
figure;
results.battery.MPGe.plot
ylabel('mpg')
title('Cyclical US06 Fuel Economy - MPGe')

%% Section 3.3: Series Hybrid Using DC Motor Genset And Higher Fidelity Battery Pack 
SOCdata = 0:10:100;
openCircuitVoltageData = [320 350 380 393 397 400 404 407 410 418 420];
tEnd = 3613;
fprintf("Running Higher Fidelity Battery Pack model on US06 Drivecycle... ")
sec33Results = sim(GRADUATE_SIMULATION_NAME_2);     results = sec33Results;
fprintf("Finished!\n")

figure;
results.battery.SOC.plot
ylabel('%')
title('US06 SOC')

figure;     hold on;
results.battery.openCircuitVoltage.plot
results.battery.terminalVoltage.plot
legend('Open Circuit Voltage [V]','Bus Voltage [V]')
title('System Voltages')
xlabel('Time [s]')
ylabel('Voltage [V]')


%% Supporting Functions
function time060 = get060(results)
    % this function finds the 0-60 time of the simulation results 'results'
    k_mph2mps = 0.4470; time = results.tout;    sampleNum = 1;
    velocity = results.vehicleDynamics.velocity.data;
    while velocity(sampleNum) < (60 * k_mph2mps)
        sampleNum = sampleNum + 1;
    end
    time060 = time(sampleNum) - 1;
end

function MPGe = getMpg(results)
    k_Ws2gal = 1/3600/1000/32.3;    k_m2mi = 0.000621371;
    energyUsed = -results.battery.batteryEnergy.data(end) * k_Ws2gal;       % [gal]
    positionFinal = results.vehicleDynamics.position.data(end) * k_m2mi;    % [mi]
    MPGe = positionFinal/energyUsed;
end

function [cycleTime, t60] = getCycleTime(results)
k_mph2mps = 0.4470; time = results.tout;    sampleNum = 1;
velocity = results.vehicleDynamics.velocity.data;
while velocity(sampleNum) < (60 * k_mph2mps)
    sampleNum = sampleNum + 1;
end
t60 = time(sampleNum);
while velocity(sampleNum) > 0
    sampleNum = sampleNum +1;
end
cycleTime = time(sampleNum);
end