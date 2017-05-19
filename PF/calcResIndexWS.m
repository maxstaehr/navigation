function [ id ] = calcResIndexWS( x, y, ny )
%CALCRESINDEXWS Summary of this function goes here
%   Detailed explanation goes here
id = double((x-1)*ny + (y-1) +1); 
end

