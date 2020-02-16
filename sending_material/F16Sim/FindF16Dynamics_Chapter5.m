 %%%%%%%%%%%%%%%%%% CHAPTER 5 %%%%%%%%%%%%%%%%%%%%%%%%%%

clear;
addpath obsmutoolsfornewermatlabversions -END % required for some new MATLAB versions
global fi_flag_Simulink
newline = sprintf('\n');

%% Trim aircraft to desired altitude and velocity
%%
xa=0/0.3048;
xa2=5*0.3048;
xa3=5.9*0.3048;
xa4=6*0.3048;
xa5=7*0.3048;
xa6=15*0.3048;
disp(sprintf('xa: %f', xa));
altitude =15000;
velocity = 500;

%% Initial guess for trim
%%
thrust = 5000;             % thrust, lbs
elevator = -0.09;          % elevator, degrees
alpha = 8.49;              % AOA, degrees
rudder = -0.01;            % rudder angle, degrees
aileron = 0.01;            % aileron, degrees

%% Find trim for Hifi model at desired altitude and velocity
%%
disp('Trimming High Fidelity Model:');
fi_flag_Simulink = 1;
[trim_state_hi, trim_thrust_hi, trim_control_hi, dLEF, xu_hi] = trim_F16(thrust, elevator, alpha, aileron, rudder, velocity, altitude);

%% Find the state space model for the hifi model at the desired alt and vel.
%%
trim_state_lin = trim_state_hi; trim_thrust_lin = trim_thrust_hi; trim_control_lin = trim_control_hi;
[A_hi,B_hi,C_hi,D_hi] = linmod('LIN_F16Block', [trim_state_lin; trim_thrust_lin; trim_control_lin(1); trim_control_lin(2); trim_control_lin(3); ...
		dLEF; -trim_state_lin(8)*180/pi], [trim_thrust_lin; trim_control_lin(1); trim_control_lin(2); trim_control_lin(3)]);

%% Find trim for Hifi model at desired altitude and velocity
%%
disp('Trimming Low Fidelity Model:');
fi_flag_Simulink = 0;
[trim_state_lo, trim_thrust_lo, trim_control_lo, dLEF, xu_lo] = trim_F16(thrust, elevator, alpha, aileron, rudder, velocity, altitude);

%% Find the state space model for the hifi model at the desired alt and vel.
%%
trim_state_lin = trim_state_lo; trim_thrust_lin = trim_thrust_lo; trim_control_lin = trim_control_lo;
[A_lo,B_lo,C_lo,D_lo] = linmod('LIN_F16Block', [trim_state_lin; trim_thrust_lin; trim_control_lin(1); trim_control_lin(2); trim_control_lin(3);...
		dLEF; -trim_state_lin(8)*180/pi], [trim_thrust_lin; trim_control_lin(1); trim_control_lin(2); trim_control_lin(3)]);

%% Make state space model
%%
SS_hi = ss(A_hi,B_hi,C_hi,D_hi);
SS_lo = ss(A_lo,B_lo,C_lo,D_lo);

%%    CHAPTER 5 
G = tf(SS_lo(19,2));
s=tf ('s');
G1 =(0.421*s^9 - 0.6734*s^8 - 23.46*s^7 - 71.02*s^6 - 229*s^5 - 359.5*s^4 - 2.766*s^3 + 0.034*s^2 + 0.0002239*s - 6.728e-16)/( s^10 + 24.5*s^9 + 102.2*s^8 + 344.8*s^7 + 765.8*s^6 + 915.2*s^5 + 685.6*s^4 + 19.19*s^3 + 4.86*s^2 + 0.05328*s - 4.078e-15);
G2 = (0.06419*s^9 - 1.888*s^8 - 27.3*s^7 - 78.86*s^6 - 232.7*s^5 - 359.5*s^4 - 2.766*s^3 + 0.034*s^2 + 0.0002239*s - 6.724e-16)/(s^10 + 24.5*s^9 + 102.2*s^8 + 344.8*s^7 + 765.8*s^6 + 915.2*s^5 + 685.6*s^4 + 19.19*s^3 + 4.86*s^2 + 0.05328*s - 4.078e-15);
G3 = (-3.138e-05*s^9 - 2.107*s^8 - 27.99*s^7 - 80.28*s^6 - 233.4*s^5 - 359.6*s^4 - 2.766*s^3 + 0.034*s^2 + 0.0002239*s - 6.731e-16)/(s^10 + 24.5*s^9 + 102.2*s^8 + 344.8*s^7 + 765.8*s^6 + 915.2*s^5 + 685.6*s^4 + 19.19*s^3 + 4.86*s^2 + 0.05328*s- 4.078e-15);
G4= (-0.007167*s^9 - 2.131*s^8 - 28.07*s^7 - 80.43*s^6 - 233.5*s^5 - 359.6*s^4 - 2.766*s^3 + 0.034*s^2 + 0.0002239*s - 6.727e-16)/(s^10 + 24.5*s^9 + 102.2*s^8 + 344.8*s^7 + 765.8*s^6 + 915.2*s^5 + 685.6*s^4 + 19.19*s^3 + 4.86*s^2 + 0.05328*s - 4.078e-15);
G5 = (-0.07853*s^9 - 2.374*s^8 - 28.84*s^7 - 82*s^6 - 234.2*s^5 - 359.6*s^4 - 2.766*s^3 + 0.034*s^2 + 0.0002239*s - 6.734e-16)/(s^10 + 24.5*s^9 + 102.2*s^8 + 344.8*s^7 + 765.8*s^6 + 915.2*s^5 + 685.6*s^4 + 19.19*s^3 + 4.86*s^2 + 0.05328*s - 4.078e-15);
G6 = (-0.6494*s^9 - 4.318*s^8 - 34.98*s^7 - 94.55*s^6 - 240.3*s^5 - 359.7*s^4 - 2.767*s^3 + 0.034*s^2 + 0.0002239*s - 6.728e-16)/(s^10 + 24.5*s^9 + 102.2*s^8 + 344.8*s^7 + 765.8*s^6 + 915.2*s^5 + 685.6*s^4 + 19.19*s^3 + 4.86*s^2 + 0.05328*s - 4.078e-15);

