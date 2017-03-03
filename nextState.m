function [ Xnext] = nextState( X , dt)

Xnext = 0*X;
for i =1:size(X,2)    
    xpos =    X(1,i);
    ypos =    X(2,i);
    theta =    X(3,i);
    vtrans =  X(4,i);
    vrot =      X(5,i);

    dx = vtrans*dt;
    dtheta = vrot * dt;
    Xnext(1,i) = cos(theta + dtheta)*dx + xpos;
    Xnext(2,i) = sin(theta + dtheta)*dx + ypos;
    Xnext(3,i) = theta + dtheta;
    Xnext(4,i) = vtrans;
    Xnext(5,i) = vrot;
end

 
end

