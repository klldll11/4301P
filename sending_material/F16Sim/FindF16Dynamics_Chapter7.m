
%%%%%%%%%%%%%%%%%% CHAPTER 7 %%%%%%%%%%%%%%%%%%%%%%%%%%

clear all
load('ABCD_full.mat') % State space system for given flight conditions
altitude = 10000; 
velocity = 900;
s=tf ('s');


%% Longitudinal Matrices

A_ac_long = A_longitude_lo([3 4 2 5], [3 4 2 5]);
B_ac_long = A_longitude_lo([3 4 2 5], [7]);

C_ac_long = C_longitude_lo([3 4 2 5], [3 4 2 5]);
D_ac_long = D_longitude_lo([3 4 2 5], [2]);


%% State space system

SS_ac_long = ss(A_ac_long, B_ac_long, C_ac_long, D_ac_long);
sys_ac_long = pck(A_ac_long, B_ac_long, C_ac_long, D_ac_long);
long_poles_ac = spoles(sys_ac_long);

%% Periodic Inherent motion characteristics 

[freq_long,damp_long] = damp(SS_ac_long);

freq_phugoid = freq_long(1);
damp_phugoid = damp_long(1);
T_phugoid = 2*pi / (freq_phugoid * sqrt(1 - damp_phugoid^2));
T_half_phugoid = log(2)/(freq_phugoid * damp_phugoid);

freq_short_period = freq_long(3);
damp_short_period = damp_long(3);
T_short_period = 2*pi / (freq_short_period * sqrt(1 - damp_short_period^2));
T_half_short_period = log(2)/(freq_short_period * damp_short_period);

dt = 0.01;       % Time vector
t = [0:dt:1000]; % Simulation time

A_shortp = A_ac_long([2 4], [2 4]);  % Reduced state-space system
B_shortp = B_ac_long([2 4], [1]);
C_shortp = C_ac_long([2 4], [2 4]);
D_shortp = D_ac_long([2 4], [1]);

SS_shortp = (1)*ss(A_shortp,B_shortp,C_shortp,D_shortp);


%% Time response comparison

dt = 0.01;
t_max = 15;
t = [0:dt:t_max];
u_elevator = [0 ones(1, size(t,2)-1)];  % Unit step inpyt elevator
y_long = lsim(SS_ac_long,u_elevator,t);
y_long_simplified = lsim(SS_shortp,u_elevator,t);

figure(7)
subplot(3,1,1);
plot(t,u_elevator,'LineWidth',1.5,'Color',[0 0.6 0.3])
ylabel({'[deg]'});
set(legend,'FontName','Helvetica','Location','Northeast');
legend('\delta_{e}');
grid on
set(findall(gcf,'-property','FontSize'),'FontSize',15)
xlim([0 t_max]);

subplot(3,1,3);
plot(t,y_long(:,[4]),'LineWidth',1.5,'color',[1 0.5 0])
hold on 
plot(t,y_long_simplified(:,2),'LineWidth',1.5,'color',[0 0.5 1])
grid on
xlabel('Time [s]');
ylabel('[deg/s]');
set(legend,'FontName','Helvetica','Location','Northeast');
legend('q, 4-state model','q, 2-state model');
xlim([0 t_max]);
set(findall(gcf,'-property','FontSize'),'FontSize',15)

subplot(3,1,2);
plot(t,y_long(:,[2]),'LineWidth',1.5,'color',[1 0.5 0])
hold on 
plot(t,y_long_simplified(:,1),'LineWidth',1.5,'color',[0 0.5 1])
grid on
xlabel('Time [s]');
ylabel('[deg]');
set(legend,'FontName','Helvetica','Location','Northeast');
legend('\alpha, 4-state model','\alpha, 2-state model');
xlim([0 t_max]);
set(findall(gcf,'-property','FontSize'),'FontSize',15)

saveas(figure(7),'Plots\comparison-2st-4st.png')


%% Pole Placement

H_q_de = tf(SS_shortp(2)); % Transfer function (TF) relating pitch rate to elevator deflection
[num_q_de,den_q_de] = tfdata(H_q_de); % Obtain numerator and denominator of TF
num_q_de = cell2mat(num_q_de);
den_q_de = cell2mat(den_q_de);
H_shortp = [H_q_de; H_q_de/s];  % TF for pitch rate and pitch angle

% open loop current properties
k_q = num_q_de(3); % Assumed to be constant, obtained from the current TF
T_tt2 = num_q_de(2)/k_q;
freq_shortp = (den_q_de(3))^0.5;
damp_shortp = den_q_de(2) / (2 * freq_shortp);
g = 9.80665; % gravitational acceleration

% required parameters (rq)
CAP_rq = g * 0.03 / 0.75;
damp_shortp_rq = 0.5;
freq_shortp_rq = 0.03 * velocity * 0.3048;
T_tt2_rq = CAP_rq * velocity *0.3048 /(g*freq_shortp_rq^2); 
DB_qs_rq = T_tt2_rq - 2 * damp_shortp_rq / freq_shortp_rq;

%using Ackermann's formula
s=tf('s');
p_alfa = (A_shortp^2) + damp_shortp_rq * freq_shortp_rq *2 * A_shortp + eye(2)* (freq_shortp_rq)^2;
C_m = [B_shortp A_shortp * B_shortp];

%Feedback matrix in degreees per radians
K_ackermann = [0 1] * C_m^(-1)*p_alfa;

A_ackermann = A_shortp-B_shortp*K_ackermann;
sys_shortp_cl = ss(A_ackermann, B_shortp,C_shortp,D_shortp); % New system with feedback matrix

lead_lag = (k_q*(1+T_tt2_rq * s))/(num_q_de(3) + num_q_de(2)*s); % Lead-lag filter
H_q_de_cl = minreal(tf(sys_shortp_cl(2)) * lead_lag);
H_aa_de_cl = minreal(tf(sys_shortp_cl(1)) * lead_lag);

H_shortp_cl = [H_q_de_cl; H_q_de_cl/s]; % Closed-loop TF for pitch rate and pitch angle
[num_q_de_cl,den_q_de_cl] = tfdata(H_q_de_cl);
[num_tt_de_cl,den_tt_de_cl] = tfdata(H_q_de_cl/s);

num_q_de_cl = cell2mat(num_q_de_cl); 
den_q_de_cl = cell2mat(den_q_de_cl);
num_tt_de_cl = cell2mat(num_tt_de_cl);
den_tt_de_cl = cell2mat(den_tt_de_cl);
freq_shortp_cl = (den_q_de_cl(3))^0.5;
damp_shortp_cl = den_q_de_cl(2) / (2 * freq_shortp_cl);
T_tt2_cl = num_q_de_cl(2)/k_q;
CAP_cl = g * freq_shortp_cl^2 * T_tt2_cl / (velocity * 0.3048); % Closed loop value for CAP


%% Time Response

dt = 0.001; % Time step
t_max = 15;
t = [0:dt:t_max];
u_step_10s = [0 ones(1,10/dt) zeros(1,(t_max-10)/dt)]; % 10s-long uit step on elevator
y_shortp = lsim(H_shortp_cl,u_step_10s,t); 

figure(8);
subplot(3,1,1);
plot(t,u_step_10s,'LineWidth',1.5,'Color',[0 0.6 0.3])
ylabel({'[deg]'});
set(legend,'FontName','Helvetica','Location','Northeast');
legend('\delta_{e}');
grid on
set(findall(gcf,'-property','FontSize'),'FontSize',15)
xlim([0 t_max]);


subplot(3,1,2);
plot(t,y_shortp(:,1),'LineWidth',1.5,'color',[1 0.5 0])
grid on
grid minor
ylabel('[deg/s]');
set(legend,'FontName','Helvetica','Location','Northeast');
legend('q');
xlim([0 t_max]);
set(gca, 'YDir','reverse') % To show the response as a 'positive' jump, easier to read
set(findall(gcf,'-property','FontSize'),'FontSize',15)

subplot(3,1,3);
plot(t,y_shortp(:,2),'LineWidth',1.5,'color',[0 0.5 1])
grid on
grid minor
xlabel('Time [s]');
ylabel('[deg]');
set(legend,'FontName','Helvetica','Location','Southeast');
legend('\theta');
xlim([0 t_max]);
set(gca, 'YDir','reverse') % To show the response as a 'positive' jump, easier to read
set(findall(gcf,'-property','FontSize'),'FontSize',15)

saveas(figure(8),'Plots\Dropback_timeseries.png')

%% CAP

figure(9);
patch([0.3 0.3 2 2 0.3],[0.085 3.6 3.6 0.085 0.085],[0.9100    0.200    0.1700],'FaceAlpha',0.3) % Creating the design regions
hold on
patch([0.2 0.2 2 2 0.25],[0.038 10 10 0.038 0.038],[0.9290, 0.640, 0.1250],'FaceAlpha',0.2)
patch([0.15 0.15 10 10 0.15],[0.01 10 10 0.01 0.01],[1,1,0.3],'FaceAlpha',0.2)
plot(damp_shortp_cl,CAP_cl,'r*','LineWidth',2)
plot(damp_shortp_cl,CAP_rq,'ks','linewidth',2,'MarkerSize',8) 
ylim([0.01 10])
xlim([0.1 10])
grid on
xlabel('\zeta_{sp} [-]');
ylabel('CAP [1/gs^2]');
legend('level 1 cat. B','level 2 cat. B','level 3 cat. B','current parameter value','design parameter value','Location','Southeast')
set(gca, 'XScale', 'log', 'YScale','log','FontSize',15)
set(findall(gcf,'-property','FontSize'),'FontSize',15)
saveas(figure(9),'Plots\CAP.png')


%% GIBSON DROPBACK CRITERION

y_shortp = lsim(-H_shortp_cl,u_step_10s,t);  % Multiplied by (-1) to match the calcualtion method in the assigment
qm_qs = max(y_shortp(:,1))/y_shortp(t==9,1); % Raio of maximum value to steady state value
DB = y_shortp(t==10,2)-y_shortp(end,2);      % Dropback
DB_qs = DB/y_shortp(t==9,1);                 % Dropback to steady state value ratio
figure(10);
patch([0 0 0.06 0.3], [1 3 3 1],[0, 0.4470, 0.7410],'FaceAlpha',0.3); % Creating the design region
hold on 
plot(DB_qs_rq,qm_qs,'ks','linewidth',2,'MarkerSize',8)
plot(DB_qs,qm_qs,'r*','linewidth',2)  
ylim([1 4])
xlim([-0.4 0.6])
grid on
xlabel('(DB/q_s) [s]');
ylabel('q_m/q_s [-]');
legend('satisfactory region','design parameter value','current parameter value')
set(gca,'FontSize',15)
set(findall(gcf,'-property','FontSize'),'FontSize',15)
saveas(figure(10),'Plots\Gibson_Dropback.png')
