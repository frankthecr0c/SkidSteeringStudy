%
% Skid steering/tracked vehicle kinematic model
% according to notation and approach in 
% Pentzer, Jesse, Sean Brennan, and Karl Reichard. 
% "Model?based Prediction of Skid?steer Robot Kinematics Using Online Estimation of Track Instantaneous Centers of Rotation." 
% Journal of Field Robotics 31.3 (2014): 455-476.
%


clc
close all
clearvars

SaveFigure = 0;
FigurePrefix = 'SkidSteering_Kinematic_Pentzer';

t0 = 0;
t_finale = 10;


% Model parameters
r = 0.30;    % [m] wheel radius

Vxr = 5;  % [rad/s] right wheel angular velocity
Vxl = 1;  % [rad/s] left wheel angular velocity

X_ICR_v = 0.2 ; % [m] x coordinate of ICR

p_skidsteering = num2cell([X_ICR_v, r, Vxl, Vxr]);

% Initial conditions
N0 = 0;            % [m]  Initial position along "north" direction in the world reference
E0 = 0;            % Initial position along "east" direction in the world reference
theta0 = 0;        % Initial angular displacement in the world reference (psi symbol in the paper
Y_ICR_r0 = 0.5;    % [m] Initial (hence, true value) y coordinate of the instantaneous center of rotation of the right side wheels/track
Y_ICR_l0 = -0.75;  % [m] Initial (hence, true value) y coordinate of the instantaneous center of rotation of the left side wheels/track
Y_ICR_v0 = -0.25;  % [m] Initial (hence, true value) y coordinate of the instantaneous center of rotation of the vehicle 

x0 = [N0; E0; theta0; Y_ICR_r0; Y_ICR_l0; Y_ICR_v0];


model = @(t, x) SkidSteering_Kinematic_Pentzer_Model(t,x,p_skidsteering);


tspan = [t0, t_finale];

%options = odeset('RelTol',1e-7,'AbsTol',1e-10);
%options = odeset('RelTol',1e-7);
%[t_ode,x_ode] = ode45(model, tspan, x0, options);
[t_ode, x_ode] = ode45(model, tspan, x0);

x_ode = x_ode';
t_ode = t_ode';

% Variable extraction

N = x_ode(1,:);        % Position along "north" direction in the world reference
E = x_ode(2,:);        % Position along "east" direction in the world reference
theta = x_ode(3,:);    % Angular displacement in the world reference (psi symbol in the paper
Y_ICR_r = x_ode(4,:);  % y coordinate of the instantaneous center of rotation of the right side wheels/track
Y_ICR_l = x_ode(5,:);  % y coordinate of the instantaneous center of rotation of the left side wheels/track
Y_ICR_v = x_ode(6,:);  % y coordinate of the instantaneous center of rotation of the vehicle 

t = t_ode;


% Drawing figures
fig10 = figure(10);
plot(N, E, 'LineWidth',2, 'DisplayName', 'Position');
hold on

title(sprintf('Vehicle planar position'))
xlabel('N [m]')
ylabel('E [m]')
grid on
legend('Location', 'East');
 

if SaveFigure
	FigureName = strcat(FigurePrefix, 'WorldPosition');
	print(fig10, '-depsc', FigureName);       
end


fig20 = figure(20);
plot(t, N, 'LineWidth',2, 'DisplayName', 'N');
hold on
plot(t, E, 'LineWidth',2, 'DisplayName', 'E');
plot(t, theta, 'LineWidth',2, 'DisplayName', '\theta');
title(sprintf('Vehicle configuration'))
xlabel('Time [s]')
ylabel('N, E [m], theta [rad]')
grid on
legend('Location', 'NorthEast');

 
