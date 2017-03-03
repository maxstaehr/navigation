
tm = GridMap(2, 2, 0.1, deg2rad(0.5));
close all; figure; hold on;  axis equal; ax1 = gca;
theta = pi/3;
line([0 cos(theta)], [0, sin(theta)]);
tm.plotMap(ax1);

N = 100;
sigma = 0.5;
sig1 = -1;
d = sqrt(2)/2;
D = repmat(d, N, 1);
T = repmat(theta, N, 1);
N = sigma*randn(N, 1);
Dn = D + N;


tm = tm. raytraceRayGrid(0,0,T, 0, Dn);
tm.plotOccupancy(ax1);
figure; hold on;  axis equal; ax2 = gca;
tm.plotProbabilityMap(ax2);

xpos = d*cos(theta);
ypos = d*sin(theta);
[xid, yid] = tm.getID(xpos, ypos);
SM = tm.getSupMap(xid, yid, 4, 4);

plot(ax1, SM(1,:),SM(2,:), 'xk');
s = [xpos-5 ypos]';
P = SM(3,:) ./ SM(4,:);
[v, idx] = max(P);
xmax = SM(1,idx);
ymax = SM(2,idx);
plot(ax1, xmax,ymax, 'xb','MarkerSize', 15);
d1 = norm(s - [xmax ymax]');
pe = exp(d1^2/sig1);
disp(pe);

scanMatcher = ScanMatcher();
pe2 = scanMatcher.matchScan([-0.2 -0.2 -deg2rad(20)], theta, d, tm);
disp(pe2);

