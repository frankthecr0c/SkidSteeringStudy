function xdot = SkidSteering_Kinematic_Pentzer_Model(t, x_vector, p_skidsteering)

%
% Skid steering/tracked vehicle kinematic model
% according to notation and approach in 
% Pentzer, Jesse, Sean Brennan, and Karl Reichard. 
% "Model?based Prediction of Skid?steer Robot Kinematics Using Online Estimation of Track Instantaneous Centers of Rotation." 
% Journal of Field Robotics 31.3 (2014): 455-476.
%


[X_ICR_v, r, u_r, u_l] = p_skidsteering{:};


N = x_vector(1);        % Position along "north" direction in the world reference
E = x_vector(2);        % Position along "east" direction in the world reference
theta = x_vector(3);    % Angular displacement in the world reference (psi symbol in the paper
Y_ICR_r = x_vector(4);  % y coordinate of the instantaneous center of rotation of the right side wheels/track
Y_ICR_l = x_vector(5);  % y coordinate of the instantaneous center of rotation of the left side wheels/track
Y_ICR_v = x_vector(6);  % y coordinate of the instantaneous center of rotation of the vehicle 

 
V_x_r_a = u_r;  % angular velocity of right wheel/track, input signal
V_x_l_a = u_l;  % angular velocity of left wheel/track, input signal

V_x_r = r*V_x_r_a;  % linear velocity of right wheel/track, input signal
V_x_l = r*V_x_l_a;  % linear velocity of left wheel/track, input signal

delta_Y = Y_ICR_l - Y_ICR_r;

% vehicle velocities
vx = (V_x_r*Y_ICR_l - V_x_l*Y_ICR_r)/delta_Y; 
vy = ((V_x_l - V_x_r)*X_ICR_v)/delta_Y; 
omega_z = -(V_x_l - V_x_r)/delta_Y; 


% Noise model, by now not used
w_N = 0;
w_E = 0;
w_theta = 0;
w_Y_ICR_r = 0;
w_Y_ICR_l = 0;  
w_Y_ICR_v = 0;



% Derivative model
c_theta = cos(theta);
s_theta = sin(theta);

N_dot = vx*c_theta - vy*s_theta + w_N;
E_dot = vx*s_theta + vy*c_theta + w_E;
theta_dot = omega_z + w_theta;
Y_ICR_r_dot = w_Y_ICR_r;
Y_ICR_l_dot = w_Y_ICR_l;
Y_ICR_v_dot = w_Y_ICR_v;



xdot=[N_dot; E_dot; theta_dot; Y_ICR_r_dot; Y_ICR_l_dot; Y_ICR_v_dot];

