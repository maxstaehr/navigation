close all; 
load('input.mat');



sigmaAccX = 1e-3;
sigmaVx = 1e-4;
sigmaVrot = 1e-4;
simgaGierrateIMU = 1e-3;
simgaGierrateOdo = 1e-3;
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
pf = PF(0, startPos);
vtrans = [1 0 0 ]';
accx = 0;
omega = 0;
x = (0:pf.dt:10)*vtrans(1);
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
h = plot(nan, nan, 'xr');
xlim([-30 30]);
ylim([-30 30]);

subplot(2, 2, 4);
axhist2 = gca;

figure; hold on;
errorPlot = gca;

figure; hold on;
occupancyPlot = gca;

counter = 0;
updateMap = false;
lastScanTime = scanTime(1);
for i=1:length(velTime)
    %% current values
    % DESCRIPTIVE TEXT
    
    vtrans = [VxRausch(i) 0 0]';
    vrot = [DRotOdoRausch(i) 0]';
    
    vtransc = [vtranscmd(i) 0 0]';
    vrotc = [vrotcmd(i) 0]';
        
    accx = AccXRausch(i);
    omega = DRotIMURausch(i);
    imuTime = velTime(i);
    %%check for update
    i1 = velTime(max([i-1 1]));
    i2 = velTime(i); 
    
    %% check for marker update
    B = intersect(find(posTime>i1),find(posTime<i2));        
    updateMarker = ~isempty(B);    
    %% check for marker update
    C = intersect(find(scanTime>i1),find(scanTime<i2));        
    updateScan = ~isempty(C);
    %%
    
    pf.predictstep(vtransc, vrotc, imuTime);
    if updateMarker && updateScan    
        
        if scanTime(C(1)) - lastScanTime > 2
            update = true;
            lastScanTime = scanTime(C(1));
        else
            update = false;
        end
        
        
        %% update marker and scan
        pos = [XposRausch(B(1)) YposRausch(B(1)) WinkelRausch(B(1))];
        posT = posTime(B(1));                                
        pf.stepMarker(vtrans, vrot, accx, omega,  pos, posT);
        %%align scan
        rp = pf.CS(1:3);   
        robot = robot.transform(rp(3), rp(1:2));
        
        disp(C(1));
        ranges = fliplr(SCAN{C(1)});
        ranges(ranges > 25) = nan;
        ranges = ranges(1:541);            
        robot = robot.setDepth(ranges);               
         
        PCL = robot.generateGlobalPCL();
        set(h, 'XData', PCL(:,1));
        set(h, 'YData', PCL(:,2));
        if update
            tm = tm.raytraceRayGridPCL(rp(1:2)', PCL);  
        end
        updateMap = true;

        
        cla(errorPlot);
        tm.plotProbabilityMap(errorPlot);
%         cla(occupancyPlot);
%         tm.plotOccupancy(occupancyPlot);
        

    elseif ~updateMarker && updateScan
        %% update marker and scan
        
        %%
        rp = pf.CS(1:3);        
        robot = robot.transform(rp(3), rp(1:2));
        ranges = fliplr(SCAN{C(1)});
        ranges(ranges > 25) = nan;
        ranges = ranges(1:541);    
        disp(C(1));
        robot = robot.setDepth(ranges);               
        
        if scanTime(C(1)) - lastScanTime > 2
            update = true;
            lastScanTime = scanTime(C(1));
        else
            update = false;
        end

        
        PCL = robot.generateGlobalPCL();        
        set(h, 'XData', PCL(:,1));
        set(h, 'YData', PCL(:,2));        
        %robot.plot(occupancyPlot);
%         cla(occupancyPlot);
%         axes(occupancyPlot);
%         plot(PCL(:,1), PCL(:,2), 'xr');
                    
        startPoses = [0 0 0];   
        cc = [0 0 0];
        if update
            [cc, score] = sm.matchScan(startPoses, PCL, tm);    
        end
        
        
        e1 = rot2(rp(3))*[cc(1) cc(2)]';
        err = bsxfun(@plus, e1 , [rp(1) rp(2)]');
        errr = rp(3) + cc(3);
        posSlam = [err' errr];              
        posT = scanTime(C(1));    
        pf.stepSlam(vtrans, vrot, accx, omega,  posSlam, posT);
        
        
        rp2 = pf.CS(1:3);   
        cc = egoKompensatePunkte(rp', [0 0 0],  rp2');
        if updateMap    
            T = rot2(cc(3))*PCL';
            T2 = bsxfun(@plus, T, [cc(1) cc(2)]');
            PCLcor = T2';     
            if update
                tm = tm.raytraceRayGridPCL(rp(1:2)', PCLcor);
            end
        end
        updateMap = true;
        if update
            tm = tm.raytraceRayGridPCL(rp(1:2)', PCL);      
        end
        cla(errorPlot);                
        tm.plotProbabilityMap(errorPlot);
%         cla(occupancyPlot);
%         tm.plotOccupancy(occupancyPlot);
        


    elseif updateMarker && ~updateScan
        %% updateMarker
        pos = [XposRausch(B(1)) YposRausch(B(1)) WinkelRausch(B(1))];
        posT = posTime(B(1));            
        pf.stepMarker(vtrans, vrot, accx, omega,  pos, posT);
    else
        %% just update ODO/IMU       
        pf.stepIMU(vtrans, vrot, accx, omega);
    end
    
    
    
    pf.plotPaticle(ax, axhist, axhist2);
    pf.plotResPos(axres);
    
end