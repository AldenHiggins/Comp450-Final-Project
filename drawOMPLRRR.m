clear all
close all
robot = RRRInit();
pit = load('printThis');
X(1,:) = pit(:,1)';
X(2,:) = pit(:,3)';
X(3,:) = pit(:,5)';
t = size(X,2);
% Graphical Simulation

robot2 = robot;
drawRRR([-1.13; -1.57; 1.57],robot2);
robot.handles = drawRRR(X(:,1),robot);
pause(3)
keyboard
for i = 2:t
    setRRR(X(:,i),robot);
    pause(1e-1); % adjustable pause in seconds
end