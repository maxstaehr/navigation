function [ vtransCorr, vrotCorr ] = berechnetVtranVrot( vl, vr, width )
%KORRIGIEREV Summary of this function goes here
%   Detailed explanation goes here



vtransCorr = (vl + vr)/2;
vrotCorr = (vr - vl)/width;

end




