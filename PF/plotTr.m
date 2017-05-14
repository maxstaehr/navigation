close all; figure; hold on; axis equal; grid on;
x = squeeze(TRAJPLANNED.data(1,1,:));
y = squeeze(TRAJPLANNED.data(1,2,:));
plot(x,y);
plot(POS.data(:,1), POS.data(:,2), 'r');
GP = POS.data;
posTime = POS.time;

%get_datalogger().addVEL((vlreal, vrreal, vl, vr, vl_limit, vr_limit, vrot, vtrans))
%dy, dtheta, dkappa, desValueRot, vRotKappa, vRotPosFiltered, vRotPosUnfiltered, vRotPos)
width = 0.525;
vlreal = VEL.data(:, 1);
vrreal = VEL.data(:, 2);
vrotcmd = VEL.data(:, 7);
vtranscmd = VEL.data(:, 8);
velTime = VEL.time;

minTime = min([min(posTime) min(velTime)]);
posTime = posTime - minTime;
velTime = velTime - minTime;

vrotreal = (vrreal - vlreal) / width;
vtransreal = (vrreal + vlreal) / 2.0;
%ay = vrotreal .* vtransreal;
ax = 0*vtransreal;
for i=2:length(ax)
    ax(i) = (vtransreal(i) - vtransreal(i-1))/(velTime(i) - velTime(i-1));  
%     ax(i) = (vtranscmd(i) - vtranscmd(i-9))/(9*0.01);  
end
save('input.mat', 'vrotreal', 'vtransreal', 'vrotcmd', 'vtranscmd', 'GP', ...
    'ax', 'velTime', 'posTime');

figure; hold on; grid on;
plot(ax);

