function [ vtransCorr, vrotCorr ] = korrigiereV( vtransreal, vrotreal , f)
%KORRIGIEREV Summary of this function goes here
%   Detailed explanation goes here
width = .525;
cmdVelRotLinks = -0.5*vrotreal*width;
cmdVelRotRechts = 0.5*vrotreal*width;
cmdVelWheelLinks = vtransreal + cmdVelRotLinks;
cmdVelWheelRechts = vtransreal + cmdVelRotRechts;
vl = f(1)*cmdVelWheelLinks;
vr = f(2)*cmdVelWheelRechts;

vtransCorr = (vl + vr)/2;
vrotCorr = (vr - vl)/width;

end

