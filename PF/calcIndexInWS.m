function [ xid, yid ] = calcIndexInWS( xmax, xmin, ymax, ymin, o, nx, ny)
%CALCINDEXINWS Summary of this function goes here
%   Detailed explanation goes here
xid = round(((o(1)-xmin)/(xmax-xmin))*(nx-1))+1;
yid = round(((o(2)-ymin)/(ymax-ymin))*(ny-1))+1;
end

