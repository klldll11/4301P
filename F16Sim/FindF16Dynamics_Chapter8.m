
%% CHAPTER 8
load('ABCD_Ch8.mat')
cost   = 4.7464e-30;
thrust = 2826.8165; % thrust, lbs
elevator   = -4.1891; % elevator, degrees
aileron    = -1.9926e-15; % aileron, degrees
rudder    = 1.2406e-14; % rudder angle, degrees
alpha  = 10.4511; % AOA, degrees
dLEF   = 0;
velocity   = 300; %ft/s
altitude = 5000; %ft
runway_altitude = 3000; %ft

Init_h = altitude - runway_altitude;
dist_levelled_flight = 3000;
 
x0= Init_h/tan(3*pi/180)+ dist_levelled_flight;

%Comment out after experiments are done
x0= x0 - 3000;

s = tf('s');

R_0 = (Init_h^2+x0^2)^0.5;

A_gs = A_longitude_lo([1 3 4 2 5], [1 3 4 2 5]);
B_gs = A_longitude_lo([1 3 4 2 5], [6, 7]);
C_gs = C_longitude_lo([1 3 4 2 5], [1 3 4 2 5]);
D_gs = D_longitude_lo([1 3 4 2 5], [1, 2]);

hdot = -5*pi;
t = 2000/(5*pi);
t_maneuver_1 = t+10;
SS_gs =ss(A_gs,B_gs,C_gs,D_gs);

%Assuming initial guess for trim, q=0 and theta=0
init_cond = [altitude, velocity, alpha, 0 ,0];

t_intercept = 9.6;
d_0 = sin(3*pi/180)*10*300;
Gamma_0 = d_0/R_0;
Slope_Ramp_Gamma = -300*sin(3*pi/180)/R_0;

%% FLARE

x1 = 1100; %ft
x2 = x1/2; %ft
tau = x1/(2*velocity); %t=3*tau
h_flare = x2*3*pi/180;
t_flare = 137.62 - 1.83; %time it takes to reach the ground from following a 3 deg glideslope - time from h_flare to the ground following a 3 deg glideslope

t_touchdown = t_flare + 3*tau; 


Gamma_f = -(x1*3/R_0)*pi/180; %rad
Slope_flare = (Gamma_f-0)/(t_touchdown-t_flare);

