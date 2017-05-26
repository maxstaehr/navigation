clear all;
close all; 
load('input.mat');
f = [1.004822203474123   1.017038207315438];
 [ vtransreal, vrotreal ] = korrigiereV( vtransreal, vrotreal , f);



sigmaAccX = 0;
sigmaVx = 0;%1e-4;
sigmaVrot =0;% 1e-4;
simgaGierrateIMU = 0;
simgaGierrateOdo = 0;
sigmaPosX =0; %1e-3;
sigmaPosY = 0;%1e-3;
sigmaPosTheta =0;% 1e-3;


Nt = length(ax);
AccXRausch = ax +             sigmaAccX * randn(Nt, 1);
DRotIMURausch = vrotreal +   simgaGierrateIMU * randn(Nt, 1);
DRotOdoRausch = vrotreal +  simgaGierrateOdo * randn(Nt, 1);
VxRausch =     vtransreal +                sigmaVx * randn(Nt, 1);


Np = length(GP(:,1));
XposRausch = GP(:,1) +          sigmaPosX * randn(Np,1);
YposRausch = GP(:,2) +          sigmaPosY * randn(Np,1);
WinkelRausch = GP(:,3) +     sigmaPosTheta * randn(Np,1);

%%
tm = GridMap(0, 50,-5, 25, 0.25, deg2rad(0.5));
robot = Robot(tm.superSamplingFaktor);
%%
maxVelTrans = 2.0;
maxVelRot = 2*pi/10;
errorFaktorTrans = 0.03;
errorFaktorRot = 0.03;

deltaSigmaTrans = errorFaktorTrans*maxVelTrans/6;
deltaSigmaRot = errorFaktorRot*maxVelRot/6;

nx = ceil(6*deltaSigmaTrans/tm.xvoxelwidth);
ny = ceil(6*deltaSigmaTrans/tm.xvoxelwidth);
sm = ScanMatcher(0.1*[deltaSigmaTrans deltaSigmaTrans deltaSigmaRot],nx, ny);

%%
startPos = GP(1,:);
vtrans = [1 0 0 ]';
accx = 0;
omega = 0;

vrot = [0 0 ]';
close all; figure; 
subplot(2, 2, 1);
hold on; grid on;
ax = gca;
subplot(2, 2, 2);
axhist = gca;

subplot(2, 2, 3);
hold on; grid on; axis equal;
axres = gca;
plot(GP(:,1), GP(:,2) , 'og');

pe = PositionEstimator(150,startPos);
h = plot(nan, nan, 'xr');
xlim([10 30]);
ylim([-10 10]);

subplot(2, 2, 4);
axhist2 = gca;

figure; hold on;
errorPlot = gca;

figure; hold on;
occupancyPlot = gca;

counter = 0;
updateMap = false;
lastScanTime = scanTime(1);
P2 = zeros(size(velTime, 1), 3);
P2(1,:) = GP(1,:);
V = zeros(7, size(velTime, 1));

pfStates = cell(1,length(velTime));

close all; figure; hold on; grid on; axis equal;
h1 = plot(nan, nan, 'og', 'MarkerSize', 25);
h2 = plot(nan, nan, 'xr');
% for i=2:length(velTime)
% 
% end

for i=2:length(velTime)
    %% current values
    % DESCRIPTIVE TEXT
    
%     vtrans = [VxRausch(i) 0 0]';
%     vrot = [DRotOdoRausch(i) 0]';
%     
%     vtransc = [vtranscmd(i) 0 0]';
%     vrotc = [vrotcmd(i) 0]';

%     accx = AccXRausch(i);
%     omega = DRotIMURausch(i);
%     imuTime = velTime(i);
    
    vtrans = [vtransreal(i) 0 0]';
    vrot = [vrotreal(i) 0]';
    accx = 0;
    omega = vrotreal(i);
    imuTime = velTime(i);
    
    vtransc = vtrans;
    vrotc = vrot;
    
    
        

    %%check for update
    i1 = velTime(max([i-1 1]));
    i2 = velTime(i); 
    
    
    %% check for marker update
    B = intersect(find(posTime>i1),find(posTime<=i2));        
    updateMarker = ~isempty(B);    
    %% check for marker update
    C = intersect(find(scanTime>i1),find(scanTime<i2));        
    updateScan = ~isempty(C);
    %%
    
    pe.predictstep(vtransc, vrotc, imuTime);
    if updateMarker   
        
%         if scanTime(C(1)) - lastScanTime > 2
%             update = true;
%             lastScanTime = scanTime(C(1));
%         else
%             update = false;
%         end
        
        
        %% update marker and scan
        pos = [XposRausch(B(1)) YposRausch(B(1)) WinkelRausch(B(1))];
        posT = posTime(B(end));               
        pe.stepIMUVel(vtrans, vrot, accx, omega);
        pe.stepMarker(pos, posT);        
        %%align scan
%         rp = pf.CS(1:3);   
%         robot = robot.transform(rp(3), rp(1:2));
        
%         disp(C(1));
%         ranges = fliplr(SCAN{C(1)});
%         ranges(ranges > 25) = nan;
%         ranges = ranges(1:541);            
%         robot = robot.setDepth(ranges);               
         
%         PCL = robot.generateGlobalPCL();
%         set(h, 'XData', PCL(:,1));
%         set(h, 'YData', PCL(:,2));
%         if update
%             tm = tm.raytraceRayGridPCL(rp(1:2)', PCL);  
%         end
%         updateMap = true;
% 
%         
%         cla(errorPlot);
%         tm.plotProbabilityMap(errorPlot);
%         cla(occupancyPlot);
%         tm.plotOccupancy(occupancyPlot);
        

    elseif ~updateMarker && updateScan
        %% update marker and scan
        
        %%
%         rp = pf.CS(1:3);        
%         robot = robot.transform(rp(3), rp(1:2));
%         ranges = fliplr(SCAN{C(1)});
%         ranges(ranges > 25) = nan;
%         ranges = ranges(1:541);    
%         disp(C(1));
%         robot = robot.setDepth(ranges);               
%         
%         if scanTime(C(1)) - lastScanTime > 2
%             update = true;
%             lastScanTime = scanTime(C(1));
%         else
%             update = false;
%         end
% 
%         
%         PCL = robot.generateGlobalPCL();        
%         set(h, 'XData', PCL(:,1));
%         set(h, 'YData', PCL(:,2));        
%         %robot.plot(occupancyPlot);
% %         cla(occupancyPlot);
% %         axes(occupancyPlot);
% %         plot(PCL(:,1), PCL(:,2), 'xr');
%                     
%         startPoses = [0 0 0];   
%         cc = [0 0 0];
%         if update
%             [cc, score] = sm.matchScan(startPoses, PCL, tm);    
%         end
%         
%         
%         e1 = rot2(rp(3))*[cc(1) cc(2)]';
%         err = bsxfun(@plus, e1 , [rp(1) rp(2)]');
%         errr = rp(3) + cc(3);
%         posSlam = [err' errr];              
%         posT = scanTime(C(1));    
%         pf.stepSlam(vtrans, vrot, accx, omega,  posSlam, posT);
%         
%         
%         rp2 = pf.CS(1:3);   
%         cc = egoKompensatePunkte(rp', [0 0 0],  rp2');
%         if updateMap    
%             T = rot2(cc(3))*PCL';
%             T2 = bsxfun(@plus, T, [cc(1) cc(2)]');
%             PCLcor = T2';     
%             if update
%                 tm = tm.raytraceRayGridPCL(rp(1:2)', PCLcor);
%             end
%         end
%         updateMap = true;
%         if update
%             tm = tm.raytraceRayGridPCL(rp(1:2)', PCL);      
%         end
%         cla(errorPlot);                
%         tm.plotProbabilityMap(errorPlot);
%         cla(occupancyPlot);
%         tm.plotOccupancy(occupancyPlot);
        


    elseif updateMarker && ~updateScan
        %% updateMarker
%         pos = [XposRausch(B(1)) YposRausch(B(1)) WinkelRausch(B(1))];
%         posT = posTime(B(1));            
%         pf.stepMarker(vtrans, vrot, accx, omega,  pos, posT);
    else
        %% just update ODO/IMU       
         pe.stepIMUVel(vtrans, vrot, accx, omega);
         
    end
    pe.recalculate();
    
    
%     pe.currentState.plotPaticle(ax, axhist, axhist2);
%     pe.currentState.plotResPos(axres);    
    P2(i,:) =  pe.currentState.CS(1:3)';
    V(:, i) = pe.currentState.varianz;
    
    pfStates{i} = pe.currentState.PRED(1:2,:);
    
%     PA = pfStates{i} ;
%     set(h2, 'XData', PA(1,:));
%     set(h2, 'YData', PA(2,:));
%     set(h1, 'XData', P2(i,1));
%     set(h1, 'YData', P2(i,2));
% %     axis equal;
%      pause(0.1);
%      disp(i);
    
    
end



% 
% 
% close all;figure; hold on; grid on;
pos = GP(1,:);
pe = PositionEstimator(150,pos);

P = zeros(size(velTime, 1), 3);
P(1,:) = GP(1,:);
% P2 = zeros(size(velTime, 1), 3);
% P2(1,:) = GP(1,:);


for i=2:length(velTime)
    dt = velTime(i) - velTime(i-1);
%     DT2(i) = dt;
    xpos = P(i-1,1);
    ypos = P(i-1, 2);
    theta = P(i-1, 3);
    
    drot = ((vrotreal(i) + vrotreal(i-1))/2)*dt;
    dtrans = ((vtransreal(i) +  vtransreal(i-1))/2)* dt;
    P(i,1) = cos(theta + drot)*dtrans + xpos;
    P(i, 2) = sin(theta + drot)*dtrans + ypos;
    P(i, 3) = theta + drot;  
    
    
    vtrans = [vtransreal(i) 0 0]';
    vrot = [vrotreal(i) 0]';
    accx = 0;
    omega = vrotreal(i);
    imuTime = velTime(i);
    
    pe.predictstep(vtrans, vrot, imuTime);
    pe.stepIMUVel(vtrans, vrot, accx, omega);
    pe.recalculate();
    P(i,:) =  pe.currentState.CS(1:3)';
    pfStates{i} = pe.currentState.PRED(1:2,:);

end
% startIndex =  1;
% endIndex=  length(velTime);
% %plot(TR(:,1), TR(:,2), 'og');
% % Ey = P(:,2) - P2(:,2);
% % plot(V(2,:));
% % plot(P(startIndex:endIndex,1), P(startIndex:endIndex,2), 'xr');
% % plot(P2(startIndex:endIndex,1), P2(startIndex:endIndex,2),'om');
% % plot(TR(:,1), TR(:,2));
% h1 = plot(nan, nan, 'og', 'MarkerSize', 25);
% h2 = plot(nan, nan, 'xr');
% for i=2:length(velTime)
%     PA = pfStates{i} ;
%     set(h2, 'XData', PA(1,:));
%     set(h2, 'YData', PA(2,:));
%     set(h1, 'XData', P(i,1));
%     set(h1, 'YData', P(i,2));
%     axis equal;
%     pause(0.1);
% end


% plot(P(1:endIndex,1)-GP(1,1), 'xr');
% plot(P2(1:endIndex,1)-GP(1,1),'om');

% plot(DT(1:endIndex), 'xr');
% plot(DT2(1:endIndex),'om');
