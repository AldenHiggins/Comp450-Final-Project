function [  ] = simulateRRR(  )
% MECH 498 - Intro to Robotics - Spring 2014
% Lab 4
% Solutions by Craig McDonald
%
%    DESCRIPTION - This function should run a dynamic simulation of the RR
%    robot for this assignment, and then play a video of the resulting
%    robot motion.
%
%    

% assumes actuator weight and uniform weighted bar



close all;

% Initialize robot
robot = RRRInit();

% Time
dt = .01; % [s]
t_f = 10; % [s]

% Numerical Integration
t = 0:dt:t_f;
n = length(t);
X = zeros(3,n); % initialize variable to hold state vector
X_dot = zeros(3,n); % initialize variable to hold state vector derivatives

% constants
m1 = robot.m_1;  % point mass of arm1 (the actuator for link 2 probably)
m2 = robot.m_2;
m3 = robot.m_3;
mr1 = robot.m_r1; % mass of link1, evenly distributed part
mr2 = robot.m_r2;
mr3 = robot.m_r3;

mr1 = 0;
mr2 = 0;  % first, for simplicity, assume that the joints are evenly distributed masses.
mr3 = 0;

l1 = robot.l_1; % length of link1
l2 = robot.l_2;
l3 = robot.l_3;

g = robot.g; % gravity

M1 = m1 + mr1;
M2 = m2 + mr2;
M3 = m3 + mr3;

cm1 = (mr1*l1/2 + m1*l1)/M1; % center of mass of link 1
cm2 = (mr2*l2/2 + m2*l2)/M2;
cm3 = (mr3*l3/2 + m3*l3)/M3;


% Initial Conditions
X_dot(:,1) = [0 0 0]';
X(:,1) = [pi/3 (pi/2 - .1) .3]; 
accel = [0 0 0]';

% Externally Applied Torque
tau_1 = 0 * ones(size(t)); % [N-m]
tau_2 = 0 * ones(size(t)); % [N-m]
for_3 = 0 * ones(size(t));

%th1subd = -pi/2;  % desired final joint angles       
th2subd = -pi/2;
Kv = 30;  % constants for impedence controllers
Kp = 130;

Iz1 = l1^3/12;
Iz2 = Iz1;


r1 = l1/2;
r2 = l2/2;


% Moments
I1 = mr1/(3 * l1) * ((l1 - cm1)^3 + cm1^3) + m1*(l1 - cm1)^2;
I2 = mr2/(3 * l2) * ((l2 - cm2)^3 + cm2^3) + m2*(l2 - cm2)^2;

E = zeros(size(t));

for i = 1:length(t)-1  % time

   th1 = X(1,i);  % joint angles
   th2 = X(2,i);
   d3  = X(3,i);


   th1d = X_dot(1,i);  % joint velocities
   th2d = X_dot(2,i);
   d3d  = X_dot(3,i);
   
   
   
   alp = Iz1 + Iz2 + M1*cm1^2 + M2*(l1^2+cm2^2);
   bet = M2*l1*r2;
   del = Iz2 + M2*cm2^2;
   
   M = [alp+2*bet*cos(th2),  del+bet*cos(th2), 0; 
        del+bet*cos(th2), del, 0;
        0, 0, M3];
    
   C = [-bet*sin(th2)*th2d,  -bet*sin(th2)*(th1d + th2d), 0;
       bet*sin(th2)*th1d,       0, 0;
       0 0 0];

   
    
     %tau(1,1) = -Kp*(th1 - th1subd) - Kv*th1d;  % applied torques
     %tau(2,1) = -Kp*(th2 - th2subd) - Kv*th2d;
     
    tau(1,1) = 3;
    tau(2,1) = 0;
    tau(3,1) = .1;
    accel_prev = accel; 
    accel = M\(tau - C*[th1d; th2d; d3d]);  % solving for acceleration
    
    % Trapezoidal Integration
    if i > 0
      X_dot(:,i+1) = X_dot(:,i) + .5 * (accel_prev + accel) * dt;  % advancing velocity
      X(:,i+1) = X(:,i) + .5 * (X_dot(:,i) + X_dot(:,i+1)) * dt; % advancing position
    end
    
    %E(i) = .5 * (M1 * (cm1 * th1d)^2 + th1d^2 * I1 + ...  % collecting total energy
     %   M2 * ( cm2^2 * th2d^2 + th2d^2 * a) + ...
      %  I2 * ((cos(th2) * th1d)^2  + th2d^2)) + ...
       % M2 * cm2 * sin(th2) * g;
    
    
end

% Graphical Simulation
keyboard
robot.handles = drawRRR(X(:,1),robot);
for i = 2:length(t)
    setRRR(X(:,i),robot);
    pause(1e-7); % adjustable pause in seconds
end

% % Plot Energy
% figure();
% plot(t,E);
% legend('Total Energy')

% Plot Output
figure();
plot(t, X(1,:), '-.');
hold on
plot(t, X(2,:));
legend('theta 1','theta 2');
ylim([-pi pi])


end
