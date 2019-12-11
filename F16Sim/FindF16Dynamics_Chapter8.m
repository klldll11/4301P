
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

s = tf('s');



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

