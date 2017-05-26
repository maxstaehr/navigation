close all; figure; hold on; axis equal; grid on;
% x = squeeze(TRAJPLANNED.data(1,1,:));
% y = squeeze(TRAJPLANNED.data(1,2,:));
plot(xTrajPlanned,yTrajPlanned);
plot(xKinexonPos, yKinexonPos, 'xr');
GP = [xKinexonPos(2:end) yKinexonPos(2:end) thetaKinexonPos(2:end)];
posTime = timeKinexonPos(2:end);
TR = [xTrajPlanned yTrajPlanned thetaTrajPlanned kappaTrajPlanned];
 
%get_datalogger().addVEL((vlreal, vrreal, vl, vr, vl_limit, vr_limit, vrot, vtrans))
%dy, dtheta, dkappa, desValueRot, vRotKappa, vRotPosFiltered, vRotPosUnfiltered, vRotPos)
width = 0.525;
vlreal =vlReal;
vrreal = vrReal;



velTime = timeVelocity;
scanTime = time;


SCAN = data;

vrotreal = (vrreal - vlreal) / width;
vtransreal = (vrreal + vlreal) / 2.0;

vrotcmd = (vrDesLim - vlDesLim) / width;
vtranscmd = (vrDesLim + vlDesLim) / 2.0;

%ay = vrotreal .* vtransreal;
ax = 0*vtransreal;
for i=2:length(ax)
    ax(i) = (vtransreal(i) - vtransreal(i-1))/(velTime(i) - velTime(i-1));  
%     ax(i) = (vtranscmd(i) - vtranscmd(i-9))/(9*0.01);  
end
save('input2.mat', 'vrotreal', 'vtransreal', 'vrotcmd', 'vtranscmd', 'GP', ...
    'ax', 'velTime', 'posTime', 'TR', 'SCAN', 'scanTime');

figure; hold on; grid on;
plot(ax);

