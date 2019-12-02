
%================================================
%     Matlab Script File used to linearize the 
%     non-linear F-16 model. The program will 
%     Extract the longitudal and lateral 
%     direction matrices.  These system matrices 
%     will be used to create pole-zero mapping
%     and the bode plots of each to each control
%     input.
% Author: Richard S. Russell
% 
%================================================
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
altitude = 10000; %input('Enter the altitude for the simulation (ft)  :  ');
velocity = 900; %input('Enter the velocity for the simulation (ft/s):  ');

%% Initial guess for trim
%%
thrust = 5000;          % thrust, lbs
elevator = -0.09;       % elevator, degrees
alpha = 8.49;              % AOA, degrees
rudder = -0.01;             % rudder angle, degrees
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


%% Make MATLAB matrix
%%
mat_hi = [A_hi B_hi; C_hi D_hi];
mat_lo = [A_lo B_lo; C_lo D_lo];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Longitudal Directional %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Select the components that make up the longitude A matrix
%%
A_longitude_hi = mat_hi([3 5 7 8 11 13 14], [3 5 7 8 11 13 14]);
A_longitude_lo = mat_lo([3 5 7 8 11 13 14], [3 5 7 8 11 13 14]);

%% Select the components that make up the longitude B matrix
%%
B_longitude_hi = mat_hi([3 5 7 8 11 13 14], [19 20]);
B_longitude_lo = mat_lo([3 5 7 8 11 13 14], [19 20]);

%% Select the components that make up the longitude C matrix
%%
C_longitude_hi = mat_hi([21 23 25 26 29], [3 5 7 8 11 13 14]);
C_longitude_lo = mat_lo([21 23 25 26 29], [3 5 7 8 11 13 14]);

%% Select the components that make up the longitude D matrix
%%
D_longitude_hi = mat_hi([21 23 25 26 29], [19 20]);
D_longitude_lo = mat_lo([21 23 25 26 29], [19 20]);

SS_long_hi = ss(A_longitude_hi, B_longitude_hi, C_longitude_hi, D_longitude_hi);
SS_long_lo = ss(A_longitude_lo, B_longitude_lo, C_longitude_lo, D_longitude_lo);

%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Lateral Directional %%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Select the components that make up the lateral A matrix
%%
A_lateral_hi = mat_hi([4 6 7 9 10 12 13 15 16], [4 6 7 9 10 12 13 15 16]);
A_lateral_lo = mat_lo([4 6 7 9 10 12 13 15 16], [4 6 7 9 10 12 13 15 16]);

%% Select the components that make up the lateral B matrix
%%
B_lateral_hi = mat_hi([4 6 7 9 10 12 13 15 16], [19 21 22]);
B_lateral_lo = mat_lo([4 6 7 9 10 12 13 15 16], [19 21 22]);

%% Select the components that make up the lateral C matrix
%%
C_lateral_hi = mat_hi([22 24 25 27 28 30], [4 6 7 9 10 12 13 15 16]);
C_lateral_lo = mat_lo([22 24 25 27 28 30], [4 6 7 9 10 12 13 15 16]);

%% Select the components that make up the lateral D matrix
%%
D_lateral_hi = mat_hi([22 24 25 27 28 30], [19 21 22]);
D_lateral_lo = mat_lo([22 24 25 27 28 30], [19 21 22]);

SS_lat_hi = ss(A_lateral_hi, B_lateral_hi, C_lateral_hi, D_lateral_hi);
SS_lat_lo = ss(A_lateral_lo, B_lateral_lo, C_lateral_lo, D_lateral_lo);

%% Make longitudal direction SYSTEM matrix
%%
sys_long_hi = pck(A_longitude_hi, B_longitude_hi, C_longitude_hi, D_longitude_hi);
sys_long_lo = pck(A_longitude_lo, B_longitude_lo, C_longitude_lo, D_longitude_lo);

%% Make lateral direction SYSTEM matrix and Find poles for hifi
%%
sys_lat_hi = pck(A_lateral_hi, B_lateral_hi, C_lateral_hi, D_lateral_hi);

long_poles_hi = spoles(sys_long_hi);
lat_poles_hi = spoles(sys_lat_hi);



%% Make lateral direction SYSTEM matrix and Find poles for lofi
%%
sys_lat_lo = pck(A_lateral_lo, B_lateral_lo, C_lateral_lo, D_lateral_lo);

long_poles_lo = spoles(sys_long_lo);
lat_poles_lo = spoles(sys_lat_lo);


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Display results


clc;



disp(sprintf('Altitude: %.3f ft.', altitude));
disp(sprintf('Velocity: %.3f ft/s\n\n', velocity));

disp('For HIFI Model:  ');
disp('Longitudal Direction:  ');
disp(newline);

disp('A =')
for i=1:length( A_longitude_hi(:,1) )
    mprintf([ A_longitude_hi(i,:) ],'  %.3e ')
end %for

disp('B =')
for i=1:length( B_longitude_hi(:,1) )
    mprintf([ B_longitude_hi(i,:) ],'  %.3e ')
end %for

disp('C =')
for i=1:length( C_longitude_hi(:,1) )
    mprintf([ C_longitude_hi(i,:) ],'  %.3e ')
end %for

disp('D =')
for i=1:length( D_longitude_hi(:,1) )
    mprintf([ D_longitude_hi(i,:) ],'  %.3e ')
end %for

rifd(long_poles_hi);

disp(newline);

disp('Lateral Direaction:  ');

disp(newline);

disp('A =')
for i=1:length( A_lateral_hi(:,1) )
    mprintf([ A_lateral_hi(i,:) ],'  %.3e ')
end %for

disp('B =')
for i=1:length( B_lateral_hi(:,1) )
    mprintf([ B_lateral_hi(i,:) ],'  %.3e ')
end %for

disp('C =')
for i=1:length( C_lateral_hi(:,1) )
    mprintf([ C_lateral_hi(i,:) ],'  %.3e ')
end %for

disp('D =')
for i=1:length( D_lateral_hi(:,1) )
    mprintf([ D_lateral_hi(i,:) ],'  %.3e ')
end %for

rifd(lat_poles_hi);

disp(newline);
disp(newline);
disp('For LOFI Model:  ');
disp('Longitudal Direction:  ');
disp(newline);

disp('A =')
for i=1:length( A_longitude_lo(:,1) )
    mprintf([ A_longitude_lo(i,:) ],'  %.3e ')
end %for

disp('B =')
for i=1:length( B_longitude_lo(:,1) )
    mprintf([ B_longitude_lo(i,:) ],'  %.3e ')
end %for

disp('C =')
for i=1:length( C_longitude_lo(:,1) )
    mprintf([ C_longitude_lo(i,:) ],'  %.3e ')
end %for

disp('D =')
for i=1:length( D_longitude_lo(:,1) )
    mprintf([ D_longitude_lo(i,:) ],'  %.3e ')
end %for

% Display the real, imaginary, frequency (magnitude) and damping ratios
rifd(long_poles_lo);

disp(newline);

disp('Lateral Direaction:  ');

disp(newline);

disp('A =')
for i=1:length( A_lateral_lo(:,1) )
    mprintf([ A_lateral_lo(i,:) ],'  %.3e ')
end %for

disp('B =')
for i=1:length( B_lateral_lo(:,1) )
    mprintf([ B_lateral_lo(i,:) ],'  %.3e ')
end %for

disp('C =')
for i=1:length( C_lateral_lo(:,1) )
    mprintf([ C_lateral_lo(i,:) ],'  %.3e ')
end %for

disp('D =')
for i=1:length( D_lateral_lo(:,1) )
    mprintf([ D_lateral_lo(i,:) ],'  %.3e ')
end %for

% Display the real, imaginary, frequency (magnitude) and damping ratios
rifd(lat_poles_lo);

%% All Poles
figure(1); 
pzmap(SS_hi, 'r', SS_lo, 'b');
title_string = sprintf('Altitude = %.2f ft Velocity = %.2f ft/s\nAll Poles\n Blue = lofi Red = hifi.', altitude, velocity);
title(title_string);
sgrid;

%% Long. Poles
%%
figure(2); 
pzmap(SS_long_hi, 'r', SS_long_lo, 'b');
title_string = sprintf('Altitude = %.2f ft Velocity = %.2f ft/s\nLongitudal Directional Poles\n Blue = lofi Red = hifi.', altitude, velocity);
title(title_string);
sgrid;

%% Lat. Poles
%%
figure(3); 
pzmap(SS_lat_hi, 'r', SS_lat_lo, 'b');
title_string = sprintf('Altitude = %.2f ft Velocity = %.2f ft/s\nLateral Directional Poles\n Blue = lofi Red = hifi.', altitude, velocity);
title(title_string);
sgrid;

% Create Bode Plots

omega = logspace(-2,2,100);

sysg_lat_hi = frsp(sys_lat_hi,omega);
sysg_lat_lo = frsp(sys_lat_lo,omega);

sysg_long_hi = frsp(sys_long_hi,omega);
sysg_long_lo = frsp(sys_long_lo,omega);

figure;
% BodeCount = 0;
% for state = 1:1:5
%     for control = 1:1:2
%         BodeCount = BodeCount +1;
%         title_string = sprintf('Bode Plot #%d\n State = %d\n Control = %d', BodeCount,state,control);
%         vplot('bode', sel(sysg_long_hi,state,control), 'b--', sel(sysg_long_lo,state,control), 'r');
%         disp(title_string);
%         legend('hifi', 'lofi');
%         pause;
%     end
% end
% 
% for state = 1:1:6
%     for control = 1:1:3
%         BodeCount = BodeCount + 1;
%         title_string = sprintf('Bode Plot #%d\n State = %d\n Control = %d', BodeCount,state,control);
%         vplot('bode', sel(sysg_lat_hi,state,control), 'b--', sel(sysg_lat_lo,state,control), 'r');
%         disp(title_string);
%         legend('hifi', 'lofi');
%         pause;
%     end
% end

%%

%WE ARE PROGRAMMING
%%%    CHAPTER 5 %%%%%%%%%%%%%%%%%%%%%%%%
G = tf(SS_lo(19,2));
s=tf ('s');
G1 =(0.421*s^9 - 0.6734*s^8 - 23.46*s^7 - 71.02*s^6 - 229*s^5 - 359.5*s^4 - 2.766*s^3 + 0.034*s^2 + 0.0002239*s - 6.728e-16)/( s^10 + 24.5*s^9 + 102.2*s^8 + 344.8*s^7 + 765.8*s^6 + 915.2*s^5 + 685.6*s^4 + 19.19*s^3 + 4.86*s^2 + 0.05328*s - 4.078e-15);
G2 = (0.06419*s^9 - 1.888*s^8 - 27.3*s^7 - 78.86*s^6 - 232.7*s^5 - 359.5*s^4 - 2.766*s^3 + 0.034*s^2 + 0.0002239*s - 6.724e-16)/(s^10 + 24.5*s^9 + 102.2*s^8 + 344.8*s^7 + 765.8*s^6 + 915.2*s^5 + 685.6*s^4 + 19.19*s^3 + 4.86*s^2 + 0.05328*s - 4.078e-15);
G3 = (-3.138e-05*s^9 - 2.107*s^8 - 27.99*s^7 - 80.28*s^6 - 233.4*s^5 - 359.6*s^4 - 2.766*s^3 + 0.034*s^2 + 0.0002239*s - 6.731e-16)/(s^10 + 24.5*s^9 + 102.2*s^8 + 344.8*s^7 + 765.8*s^6 + 915.2*s^5 + 685.6*s^4 + 19.19*s^3 + 4.86*s^2 + 0.05328*s- 4.078e-15);
G4= (-0.007167*s^9 - 2.131*s^8 - 28.07*s^7 - 80.43*s^6 - 233.5*s^5 - 359.6*s^4 - 2.766*s^3 + 0.034*s^2 + 0.0002239*s - 6.727e-16)/(s^10 + 24.5*s^9 + 102.2*s^8 + 344.8*s^7 + 765.8*s^6 + 915.2*s^5 + 685.6*s^4 + 19.19*s^3 + 4.86*s^2 + 0.05328*s - 4.078e-15);
G5 = (-0.07853*s^9 - 2.374*s^8 - 28.84*s^7 - 82*s^6 - 234.2*s^5 - 359.6*s^4 - 2.766*s^3 + 0.034*s^2 + 0.0002239*s - 6.734e-16)/(s^10 + 24.5*s^9 + 102.2*s^8 + 344.8*s^7 + 765.8*s^6 + 915.2*s^5 + 685.6*s^4 + 19.19*s^3 + 4.86*s^2 + 0.05328*s - 4.078e-15);
G6 = (-0.6494*s^9 - 4.318*s^8 - 34.98*s^7 - 94.55*s^6 - 240.3*s^5 - 359.7*s^4 - 2.767*s^3 + 0.034*s^2 + 0.0002239*s - 6.728e-16)/(s^10 + 24.5*s^9 + 102.2*s^8 + 344.8*s^7 + 765.8*s^6 + 915.2*s^5 + 685.6*s^4 + 19.19*s^3 + 4.86*s^2 + 0.05328*s - 4.078e-15);

%%%%%%%%%%%%%%%%%% CHAPTER 6 %%%%%%%%%%%%%%%%%%%%%%%%%%

%Longitudinal Matrices

A_ac_long = A_longitude_lo([3 4 2 5], [3 4 2 5]);
B_ac_long = A_longitude_lo([3 4 2 5], [7]);

C_ac_long = C_longitude_lo([3 4 2 5], [3 4 2 5]);
D_ac_long = D_longitude_lo([3 4 2 5], [2]);


%Lateral Matrices

A_ac_lat = A_lateral_lo([4 1 5 6], [4 1 5 6]);
B_ac_lat = A_lateral_lo([4 1 5 6], [8 9]);

C_ac_lat = C_lateral_lo([4 1 5 6], [4 1 5 6]);
D_ac_lat = D_lateral_lo([4 1 5 6], [2 3]);


%% State space system

SS_ac_long = ss(A_ac_long, B_ac_long, C_ac_long, D_ac_long);
sys_ac_long = pck(A_ac_long, B_ac_long, C_ac_long, D_ac_long);

SS_ac_lat = ss(A_ac_lat, B_ac_lat, C_ac_lat, D_ac_lat);
sys_ac_lat = pck(A_ac_lat, B_ac_lat, C_ac_lat, D_ac_lat);

long_poles_ac = spoles(sys_ac_long);
lat_poles_ac = spoles(sys_ac_lat);

figure(4); 
pzmap(SS_ac_long, 'r', SS_ac_lat, 'b');
title_string = sprintf('Altitude = %.2f ft Velocity = %.2f ft/s\nAll Poles\n Blue = lateral Red = longitudinal.', altitude, velocity);
title(title_string);
sgrid;

%% Periodic Inherent motion characteristics 
%%

[freq_long,damp_long] = damp(SS_ac_long);
[freq_lat,damp_lat,poles_lat] = damp(SS_ac_lat);

freq_phugoid = freq_long(1);
damp_phugoid = damp_long(1);
T_phugoid = 2*pi / (freq_phugoid * sqrt(1 - damp_phugoid^2));
T_half_phugoid = log(2)/(freq_phugoid * damp_phugoid);

freq_short_period = freq_long(3);
damp_short_period = damp_long(3);
T_short_period = 2*pi / (freq_short_period * sqrt(1 - damp_short_period^2));
T_half_short_period = log(2)/(freq_short_period * damp_short_period);

freq_dutch_roll = freq_lat(3);
damp_dutch_roll = damp_lat(3);
T_half_dutch_roll = log(2)/(freq_dutch_roll * damp_dutch_roll);

dt = 0.01;
t = [0:dt:30];
y_long = step(SS_ac_long,t);
u_aileron = [ones(size(t))];
y_lat = step(SS_ac_lat,t);

%%
% % plot phugoid
% figure; 
% plot(t,y_long(:,[1 3 4]))
% legend("speed","pitch angle","pitch rate")
% grid on
% 
% plot short period
figure(6); 
plot(t,y_long(:,[2 4]))
legend("aoa","pitch rate")
grid on
% 
% %plot dutch roll
% figure;
% plot(t,y_lat(:,[3 4],1))
% legend("roll rate","yaw rate")
% grid on

%% Aperiodic Inherent motion characteristics 
%%
% 
% time_const_spiral = - 1 / poles_lat(1);
% time_const_ap_roll = - 1 / poles_lat(2);
% 
% %plot spiral
% figure;
% plot(t,y_lat(:,[2 4],2))
% legend("roll angle","yaw rate")
% grid on
% 
% % plot aperiodic roll
% figure;
% plot(t,y_lat(:,[2 3],1))
% legend("roll angle","roll rate")
% grid on

%%

A_shortp = A_ac_long([2 4], [2 4]);
B_shortp = B_ac_long([2 4], [1]);
C_shortp = C_ac_long([2 4], [2 4]);
D_shortp = D_ac_long([2 4], [1]);

SS_shortp=ss(A_shortp,B_shortp,C_shortp,D_shortp);
y=step(SS_shortp,t);
% figure(5)
% plot(t,y);
% legend("aoa","pitch rate");
% grid on

H_shortp = tf(SS_shortp);
H_q_de = H_shortp(2);
[num_q_de,den_q_de] = tfdata(SS_shortp(2,1));
num_q_de = cell2mat(num_q_de);
den_q_de = cell2mat(den_q_de);

H_aa_de = tf(SS_shortp(1,1));

% open loop current properties
k_q = num_q_de(3); %Assumed to be constant
T_tt2 = num_q_de(2)/k_q;
freq_shortp = (den_q_de(3))^0.5;
damp_shortp = den_q_de(2) / (2 * freq_shortp);
g = 9.80665;
CAP = g * freq_shortp^2 * T_tt2 / (velocity * 0.3048);


% required parameters (rq)
CAP_rq = g * 0.03 / 0.75;
damp_shortp_rq = 0.5;
freq_shortp_rq = 0.03 * velocity * 0.3048;
T_tt2_rq = CAP_rq * velocity *0.3048 /(g*freq_shortp_rq^2); 
DB_qs_rq = T_tt2_rq - 2 * damp_shortp_rq / freq_shortp_rq;

%using Ackermann's formula
s=tf('s');
p_alfa = (A_shortp^2) + damp_shortp_rq * freq_shortp_rq *2 * A_shortp + eye(2)* (freq_shortp_rq)^2;
C_m=[B_shortp A_shortp * B_shortp];

%Feedback matrix in degreees per radians
K_ackermann = [0 1] * C_m^(-1)*p_alfa;

A_ackermann = A_shortp-B_shortp*K_ackermann;
sys_shortp_cl = ss(A_ackermann, B_shortp,C_shortp,D_shortp);

%damp(sys_shortp_cl);

lead_lag=(k_q*(1+T_tt2_rq * s))/(num_q_de(3)+num_q_de(2)*s);
H_q_de_cl = minreal(tf(sys_shortp_cl(2)) * lead_lag);
H_aa_de_cl = minreal(tf(sys_shortp_cl(1)) * lead_lag);

H_shortp_cl = [H_q_de_cl; H_q_de_cl/s];

%% Close Loop Performance Evaluation
[num_q_de_cl,den_q_de_cl] = tfdata(H_q_de_cl);

num_q_de_cl = cell2mat(num_q_de_cl);
den_q_de_cl = cell2mat(den_q_de_cl);
freq_shortp_cl = (den_q_de_cl(3))^0.5;
damp_shortp_cl = den_q_de_cl(2) / (2 * freq_shortp_cl);
T_tt2_cl = num_q_de_cl(2)/k_q;
CAP_cl = g * freq_shortp_cl^2 * T_tt2_cl / (velocity * 0.3048);

%% TIME RESPONSE
dt = 0.01;
t = [0:dt:30];
u_step_10s = [ones(1,10/dt+1) zeros(1,(30-10)/dt)];

y=lsim(-H_shortp_cl,u_step_10s,t);  %Explain why negative

figure(7);
plot(t,y,'linewidth',2) 
hold on
plot(t,u_step_10s,'linewidth',2) 
%gamma_0 = y_long(1,3) - y_long(1,2)
legend('pitch rate','pitch attitude','input','location','northwest')
grid on
grid minor
xlabel('time [s]');
ylabel('[\circ] and [\circ/s]');
title('Input and Short period response','FontWeight','Normal')
set(gca,'FontSize',15)

%% PLOT CAP
figure(8);
patch([0.3 0.3 1.5 1.5 0.3],[0.28 3.6 3.6 0.28 0.28],[0.9100    0.200    0.1700],'FaceAlpha',0.2)
hold on
patch([0.25 0.25 2 2 0.25],[0.16 10 10 0.16 0.16],[0.9290, 0.640, 0.1250],'FaceAlpha',0.2)
patch([0.125 0.125 10 10 0.125],[0.01 10 10 0.01 0.01],[1,1,0.3],'FaceAlpha',0.2)
%xline(0.125,'linewidth',2);
plot(damp_shortp_cl,CAP_cl,'r*','linewidth',2) 
ylim([0.01 10])
xlim([0.1 10])
grid on
xlabel('\zeta_{sp} [-]');
ylabel('CAP [1/gs^2]');
legend('level 1 cat. A','level 2 cat. A','level 3 cat. A','current parameter value','Location','Southeast')
title('Control Anticipation Parameter versus \zeta_{sp}','FontWeight','Normal')
set(gca, 'XScale', 'log', 'YScale','log','FontSize',15)


%% GIBSON DROPBACK CRITERION
qm_qs = max(y(:,2))/y(end,2);
DB_qs = (y(t==10,2)-y(end,2))/y(end,2);
figure(10);
patch([0 0 0.06 0.3], [1 3 3 1],[0, 0.4470, 0.7410],'FaceAlpha',0.3);
hold on 
plot(DB_qs,qm_qs,'r*','linewidth',2)  
plot(DB_qs_rq,qm_qs,'k*','linewidth',2)
ylim([1 4])
xlim([-0.4 0.6])
title({'Gibson Dropback Criterion'},'FontWeight','Normal')
grid on
xlabel('(DB/q_s) [s]');
ylabel('q_m/q_s [-]');
legend('satisfactory region','current parameter value','design parameter value')
set(gca,'FontSize',15)


%% GIBSON RATE CRITERION
[G_Wcp,P_Wcp,Wcg,Wcp] = margin(H_q_de*lead_lag); %Change TF to cl
[G_2Wcp,P_2Wcp] = bode(H_q_de*lead_lag,2*Wcp);
phase_rate = -(P_2Wcp-P_Wcp)/(Wcp);
figure(9);
l = linspace(0, 2*pi, 200);
x = 0.3 * cos(l) + 1.2;
y = 20 * sin(l) + 75;
patch(x, y, [0, 0.4470, 0.7410],'FaceAlpha',0.3);
hold on 
plot(DB_q_ss_cl,2.5,'r*','linewidth',2)  %check qm/qs
yline(100,'--k');
ylim([0 300])
xlim([0 2])
title({'Gibson Rate Criterion'},'FontWeight','Normal')
grid on
xlabel('-180^{\circ} phase lag frequency [Hz]');
ylabel('phase rate [^{\circ}/Hz]');
legend('optimal region','current parameter value')
set(gca,'FontSize',15);

