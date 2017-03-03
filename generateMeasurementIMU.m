function [ IMU ] = generateMeasurementIMU(X, XP, dt )
%UNTITLED5 Summary of this function goes here
%   Detailed explanation goes here
IMU = zeros(2,size(X,2));
for i =1:size(X,2)    
    %% find distance moved
    s = norm(XP(1:2, i) - X(1:2,i));
    a = (2*(s-XP(4,i)*dt))/dt^2;
    vrot = XP(4,i);
    
    IMU(1,i) = a;
    IMU(2,i) = vrot;
end

end

