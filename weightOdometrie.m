function [ w ] = weightOdometrie(X,  v, var )
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
vartrans = var(1);
varrot = var(2);
measvtrans = v(1);
measvrot = v(2);

w = zeros(1,size(X,2));
for i =1:size(X,2)    
    vtrans =  X(4,i);
    vrot =      X(5,i);
           
    %Generate the weights for each of these particles.
    %The weights are based upon the probability of the given
    %observation for a particle, GIVEN the actual observation.
    %That is, if we observe a location z, and we know our observation error is
    %guassian with variance x_R, then the probability of seeing a given
    %z centered at that actual measurement is (from the equation of a
    %gaussian)
    wtrans = (1/sqrt(2*pi*vartrans)) * exp(-(measvtrans - vtrans)^2/(2*vartrans));
    wrot = (1/sqrt(2*pi*varrot)) * exp(-(measvrot - vrot)^2/(2*varrot));     
    w(i) = wtrans + wrot;    
end
end
