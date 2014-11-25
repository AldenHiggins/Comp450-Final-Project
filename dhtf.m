function A = dhtf(alpha,a,d,theta)

X = screwtf(a,alpha,[1,0,0]');
Z = screwtf(d,theta,[0,0,1]');

A = X*Z;

end

% translation and rotation about single axis
function T = screwtf(trans,theta,ax)

v = ax/norm(ax);

crossMat = [0 -v(3) v(2);
            v(3) 0 -v(1);
            -v(2) v(1) 0];
        
R = cos(theta) * eye(3) + (1 - cos(theta)) * v*(v') + sin(theta) * crossMat;

T = [R, trans * v];
T = [T; [0,0,0,1]];

end
