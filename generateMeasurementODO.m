function [ ODO ] = generateMeasurementODO(X)
%UNTITLED5 Summary of this function goes here
%   Detailed explanation goes here
ODO = zeros(2,size(X,2));
for i =1:size(X,2)    
    ODO(1,i) = X(4,i);
    ODO(2,i) = X(5,i);
end

end