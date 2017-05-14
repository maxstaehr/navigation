function [ PR ] = normalizeP( P )
%UNTITLED7 Summary of this function goes here
%   Detailed explanation goes here
sumP = sum(P);
if sumP > 0
    PR = P./sumP;    
else
    PR = P;    
end

end

