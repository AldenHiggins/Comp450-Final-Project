function setRRR( angles, robot )
% MECH 498 - Intro to Robotics - Spring 2014
% Lab 4
% Solutions by Craig McDonald
% 
%    DESCRIPTION - Update the position of the robot after calling drawRR()
% 
%    This function can be used as is once RRFK() and drawRR() have been
%    completed.

[~,robot_T] = RRRFK(angles,robot);
set(robot.handles(1),'Matrix',robot_T{1});
set(robot.handles(2),'Matrix',robot_T{2});
set(robot.handles(3),'Matrix',robot_T{3});
drawnow;

end

