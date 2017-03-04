
LL = {};

L = [-15 30 ;    
    
    -15 20 ;
    -16 20 ;
    -16 30 ;
    -45 30 ;
    
    
    -45 -15 ;
    -16 -15 ;
    -16 0 ;
    -15 0 ;

    
      -15 -15 ;
        15 -15;
        15 30;
        -15 30];

LL{end+1} = L;

L = [-.5 .5 ;
       -.5 -.5 ;
        .5 -.5;
        .5 .5;
        -.5 .5];   

LL{end+1} = bsxfun(@plus, [5 -10], L);
LL{end+1} = bsxfun(@plus, [5 -5], L);
LL{end+1} = bsxfun(@plus, [5 0], L);
LL{end+1} = bsxfun(@plus, [5 5], L);
LL{end+1} = bsxfun(@plus, [5 10], L);
LL{end+1} = bsxfun(@plus, [5 15], L);
LL{end+1} = bsxfun(@plus, [5 20], L);
LL{end+1} = bsxfun(@plus, [5 25], L);
    
env = Environment(LL);



robot = Robot();
sm = ScanMatcher();






close all;figure; hold on; grid on; axis equal;
ax = gca;
env.plot(ax);
figure; hold on; grid on; axis equal;
ax_map = gca;
figure; hold on; grid on; axis equal;
ax_prob = gca;


tm = GridMap(-50, 20,-20, 35, 0.5, deg2rad(0.5));
tmax = 15;
tdelta = 0.05;
T = 0:tdelta:tmax;
Y = T*2-10;

S = struct();
S.fA = robot.frontScanner.A;
S.rA = robot.rearScanner.A;

D = cell(1,length(Y));
for i=1:length(Y)
    robotPose = [0 Y(i) pi/2];
    
    robot = robot.transform(pi/2, [0 Y(i)]');
    robot = robot.raytrace(env);
    
%     cla(ax);
%     cla(ax_map);
%     cla(ax_prob);
%     cla(ax);
%     
    xf = robot.getFrontLaserTransformation();
%     tm = tm. raytraceRayGrid(xf(1),xf(2),xf(3), robot.frontScanner.A, robot.frontScanner.D);
    xr = robot.getRearLaserTransformation();
%     tm = tm. raytraceRayGrid(xr(1),xr(2),xr(3), robot.rearScanner.A, robot.rearScanner.D);
%     tm.plotProbabilityMap(ax_prob);
    
%     n = [0.05*randn 0.05*randn deg2rad(5)*randn];
%     xf = xf + n;
%     xr = xr + n;
    
    s = struct();
    s.fD = robot.frontScanner.D;
    s.rD = robot.rearScanner.D;
    s.xf = xf;
    s.xr = xr;
    D{end+1} = s;
    
    
    
%     At = robot.frontScanner.A + xf(3);
%     XPOS = robot.frontScanner.D .*cos(At) + xf(1);
%     YPOS = robot.frontScanner.D .*sin(At) + xf(2);
%     PCL = [XPOS' YPOS'];
%     
%     At = robot.rearScanner.A + xr(3);
%     XPOS = robot.rearScanner.D .*cos(At) + xr(1);
%     YPOS = robot.rearScanner.D .*sin(At) + xr(2);
%     PCL = vertcat(PCL, [XPOS' YPOS']);
%     axes(ax);
%     env.plot(ax);
%     plot(PCL(:, 1), PCL(:,2), 'xr');
%     c = sm.matchScan([0 0 0], PCL, tm);
%     
%     
%     T = rot2(c(3))*PCL';
%     T2 = bsxfun(@plus, T, [c(1) c(2)]');
%     T3 = T2';
% 
%     axes(ax);
%     plot(T3(:, 1), T3(:,2), 'og');
%   
%     
%     
%     robot.plot(ax);
%     tm.plotOccupancy(ax_map);
%     tm.plotProbabilityMap(ax_prob);
    disp(i);
end
save('laserScanData.mat', 'D', 'S');


