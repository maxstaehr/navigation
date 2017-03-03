function [ w ] = weightIMU(X, Xlast,  v, var, dt )
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
varax = var(1);
varrot = var(2);

measax = v(1);
measvrot = v(2);

w = zeros(1,size(X,2));
for i =1:size(X,2)    
    ax   =  (X(4,i) - Xlast(4,i))/dt;
    vrot =   X(5,i);
           
    %Generate the weights for each of these particles.
    %The weights are based upon the probability of the given
    %observation for a particle, GIVEN the actual observation.
    %That is, if we observe a location z, and we know our observation error is
    %guassian with variance x_R, then the probability of seeing a given
    %z centered at that actual measurement is (from the equation of a
    %gaussian)
    wax = (1/sqrt(2*pi*varax)) * exp(-(measax - ax)^2/(2*varax));
    wrot = (1/sqrt(2*pi*varrot)) * exp(-(measvrot - vrot)^2/(2*varrot));     
    w(i) = wax + wrot;    
end
end

