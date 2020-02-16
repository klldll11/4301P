
%%%%%%%%%%%%%%%%%% CHAPTER 6 %%%%%%%%%%%%%%%%%%%%%%%%%%

clear all
load('ABCD_full.mat')
altitude = 10000; %input('Enter the altitude for the simulation (ft)  :  ');
velocity = 900; %input('Enter the velocity for the simulation (ft/s):  ');
s=tf ('s');


%% Longitudinal Matrices

A_ac_long = A_longitude_lo([3 4 2 5], [3 4 2 5]);
B_ac_long = A_longitude_lo([3 4 2 5], [7]);

C_ac_long = C_longitude_lo([3 4 2 5], [3 4 2 5]);
D_ac_long = D_longitude_lo([3 4 2 5], [2]);


%% Lateral Matrices

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

% figure(4); 
% pzmap(SS_ac_long, 'r', SS_ac_lat, 'b');
% 
% title('All Poles\n Blue = lateral Red = longitudinal.');
% sgrid;

%% Periodic Inherent motion characteristics 

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
T_dutch_roll = 2*pi / (freq_dutch_roll * sqrt(1 - damp_dutch_roll^2));
T_half_dutch_roll = log(2)/(freq_dutch_roll * damp_dutch_roll);

time_const_spiral = - 1 / poles_lat(1);
time_const_ap_roll = - 1 / poles_lat(4);

dt = 0.01;
t = [0:dt:1000];
u_elevator = [0 ones(1, size(t,2)-1)];
y_long = lsim(SS_ac_long,u_elevator,t);
u_aileron = [0 ones(1, size(t,2)-1); zeros(size(t))];
u_rudder =  [zeros(size(t)); 0 ones(1, size(t,2)-1)];
u_rudder_10 = [zeros(size(t)); 0, ones(1, 10/dt-1), zeros(1, size(t,2)-10/dt)];
y_lat_aileron = lsim(SS_ac_lat,u_aileron,t);
y_lat_rudder = lsim(SS_ac_lat,u_rudder,t);
y_lat_rudder_10 = lsim(SS_ac_lat,u_rudder_10,t);


%% SHORT PERIOD

figure(1);
subplot(4,1,1);
plot(t,u_elevator,'LineWidth',1.5,'Color',[0.2 0.8 0.8])
ylabel('[deg]');
set(legend,'FontName','Helvetica','Location','Northeast');
legend('\delta_{e}');
grid on
set(findall(gcf,'-property','FontSize'),'FontSize',15)
xlim([0 3]);

subplot(4,1,2);
plot(t,y_long(:,[2]),'LineWidth',1.5,'color',[1 0.5 0])
grid on
ylabel('[deg]');
xlim([0 3]);
set(legend,'FontName','Helvetica','Location','Northeast');
legend('\alpha');
set(findall(gcf,'-property','FontSize'),'FontSize',15)

subplot(4,1,3);
plot(t,y_long(:,[3]),'LineWidth',1.5,'color',[0 0.5 1])
grid on
ylabel('[deg]');
xlim([0 3]);
set(legend,'FontName','Helvetica','Location','Northeast');
legend('\theta');
set(findall(gcf,'-property','FontSize'),'FontSize',15)

subplot(4,1,4);
plot(t,y_long(:,[4]),'LineWidth',1.5,'color',[0 0.6 0.3])
grid on
xlabel('Time [s]');
ylabel('[deg/s]');
set(legend,'FontName','Helvetica','Location','Northeast');
legend('q');
xlim([0 3]);
set(findall(gcf,'-property','FontSize'),'FontSize',15)

saveas(figure(1),'Plots\shortp.png')

%% PHUGOID

figure(2)
subplot(4,1,1);
plot(t,u_elevator,'LineWidth',1.5,'Color',[0.2 0.8 0.8])
ylabel('[deg]');
set(legend,'FontName','Helvetica','Location','Northeast');
legend('\delta_{e}');
grid on
set(findall(gcf,'-property','FontSize'),'FontSize',15)
xlim([0 500]);

subplot(4,1,2);
plot(t,y_long(:,[1]),'LineWidth',1.5,'color',[1 0.5 0])
grid on
ylabel('[ft/s]');
xlim([0 500]);
set(legend,'FontName','Helvetica','Location','Southeast');
legend('V');
set(findall(gcf,'-property','FontSize'),'FontSize',15)

subplot(4,1,3);
plot(t,y_long(:,[3]),'LineWidth',1.5,'color',[0 0.5 1])
grid on
ylabel('[deg]');
xlim([0 500]);
set(legend,'FontName','Helvetica','Location','Southeast');
legend('\theta');
set(findall(gcf,'-property','FontSize'),'FontSize',15)

subplot(4,1,4);
plot(t,y_long(:,[4]),'LineWidth',1.5,'color',[0 0.6 0.3])
grid on
xlabel('Time [s]');
ylabel('[deg/s]');
set(legend,'FontName','Helvetica','Location','Southeast');
legend('q');
xlim([0 500]);
set(findall(gcf,'-property','FontSize'),'FontSize',15)

saveas(figure(2),'Plots\phugoid.png')

%% DUTCH ROLL

figure(3)
subplot(3,1,1);
plot(t,u_rudder(1,:),'LineWidth',1.5,'Color',[0 0.6 0.3])
hold on
plot(t,u_rudder(2,:),'LineWidth',1.5,'Color',[0.2 0.8 0.8])
ylabel({'[deg]'});
set(legend,'FontName','Helvetica','Location','Northeast');
legend('\delta_{a}','\delta_r');
grid on
set(findall(gcf,'-property','FontSize'),'FontSize',15)
xlim([0 8]);

subplot(3,1,2);
plot(t,y_lat_rudder(:,[3]),'LineWidth',1.5,'color',[1 0.5 0])
grid on
ylabel('[deg/s]');
xlim([0 8]);
set(legend,'FontName','Helvetica','Location','Northeast');
legend('p');
set(findall(gcf,'-property','FontSize'),'FontSize',15)

subplot(3,1,3);
plot(t,y_lat_rudder(:,[4]),'LineWidth',1.5,'color',[0 0.5 1])
grid on
ylabel('[deg/s]');
xlim([0 8]);
set(legend,'FontName','Helvetica','Location','Northeast');
legend('r');
set(findall(gcf,'-property','FontSize'),'FontSize',15)

saveas(figure(3),'Plots\dutchroll.png')

%% SPIRAL

figure(4)
subplot(3,1,1);
plot(t,u_rudder_10(1,:),'LineWidth',1.5,'Color',[0 0.6 0.3])
hold on
plot(t,u_rudder_10(2,:),'LineWidth',1.5,'Color',[0.2 0.8 0.8])
ylabel({'[deg]'});
set(legend,'FontName','Helvetica','Location','Northeast');
legend('\delta_{a}','\delta_r');
grid on
set(findall(gcf,'-property','FontSize'),'FontSize',15)
xlim([0 60]);

subplot(3,1,2);
plot(t,y_lat_rudder_10(:,[2]),'LineWidth',1.5,'color',[1 0.5 0])
grid on
ylabel('[deg]');
xlim([0 60]);
set(legend,'FontName','Helvetica','Location','Northeast');
legend('\phi');
set(findall(gcf,'-property','FontSize'),'FontSize',15)

subplot(3,1,3);
plot(t,y_lat_rudder_10(:,[4]),'LineWidth',1.5,'color',[0 0.5 1])
grid on
ylabel('[deg/s]');
xlim([0 60]);
set(legend,'FontName','Helvetica','Location','Northeast');
legend('r');
set(findall(gcf,'-property','FontSize'),'FontSize',15)

saveas(figure(4),'Plots\spiral.png')

%% APERIODIC ROLL

figure(5)
subplot(3,1,1);
plot(t,u_aileron(1,:),'LineWidth',1.5,'Color',[0 0.6 0.3])
hold on
plot(t,u_aileron(2,:),'LineWidth',1.5,'Color',[0.2 0.8 0.8])
ylabel({'[deg]'});
set(legend,'FontName','Helvetica','Location','Northeast');
legend('\delta_{a}','\delta_r');
grid on
set(findall(gcf,'-property','FontSize'),'FontSize',15)
xlim([0 7]);

subplot(3,1,2);
plot(t,y_lat_aileron(:,[2]),'LineWidth',1.5,'color',[1 0.5 0])
grid on
ylabel('[deg]');
xlim([0 7]);
set(legend,'FontName','Helvetica','Location','Northeast');
legend('\phi');
set(findall(gcf,'-property','FontSize'),'FontSize',15)

subplot(3,1,3);
plot(t,y_lat_aileron(:,[3]),'LineWidth',1.5,'color',[0 0.5 1])
grid on
ylabel('[deg/s]');
xlim([0 7]);
set(legend,'FontName','Helvetica','Location','Northeast');
legend('p');
set(findall(gcf,'-property','FontSize'),'FontSize',15)

saveas(figure(5),'Plots\aperiodicroll.png')


