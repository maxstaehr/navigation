function [ Xnext] = nextStateIMU( X , v, dt)
%NEXTSTATEVELOCITY Summary of this function goes here
%   Detailed explanation goes here

Xnext = 0*X;
for i =1:size(X,2)    
    xpos =    X(1,i);
    ypos =    X(2,i);
    theta =    X(3,i);
    vtrans = X(4,i);
    
    ax =  v(1);
    vrot =      v(2);

    dx = 0.5*ax*dt^2 + vtrans*dt;
    dtheta = vrot * dt;
    
    Xnext(1,i) = cos(theta + dtheta)*dx + xpos;
    Xnext(2,i) = sin(theta + dtheta)*dx + ypos;
    Xnext(3,i) = theta + dtheta;
    Xnext(4,i) = vtrans + ax*dt;
    Xnext(5,i) = vrot;
end

 
end
