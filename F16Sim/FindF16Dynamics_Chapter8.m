
%% CHAPTER 8
load('ABCD_Ch8.mat')
altitude = 5000; %input('Enter the altitude for the simulation (ft)  :  ');
velocity = 300; %input('Enter the velocity for the simulation (ft/s):  ');
thrust = 5000;          % thrust, lbs
elevator = -0.09;       % elevator, degrees
alpha = 8.49;              % AOA, degrees
rudder = -0.01;             % rudder angle, degrees
aileron = 0.01;            % aileron, degrees
s = tf('s');



A_gs = A_longitude_lo([1 3 4 2 5], [1 3 4 2 5]);
B_gs = A_longitude_lo([1 3 4 2 5], [6, 7]);
C_gs = C_longitude_lo([1 3 4 2 5], [1 3 4 2 5]);
D_gs = D_longitude_lo([1 3 4 2 5], [1, 2]);

hdot = -5*pi;
t = 2000/(5*pi);
 SS_gs =ss(A_gs,B_gs,C_gs,D_gs);
% C = pidTuner (SS_gs);

Q=[1/1000 0 0 0 0
    0 1/(90) 0 0 0
    0 0 1 0 0
    0 0 0 1/4 0
    0 0 0 0 1];
R=[1/25 0
    0  1/15];

K=lqr(SS_gs,Q,R)
KA=K([1 2],[1 2])
KB=K([1 2],[3 4 5])

%Assuming initial guess for trim, q=0 and theta=0
init_cond = [altitude, velocity, alpha, 0 ,0];

