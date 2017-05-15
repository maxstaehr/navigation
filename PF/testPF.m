close all; clear all;
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
tm = GridMap(-50, 20,-35, 35, 0.05, deg2rad(0.5));
robot = Robot(tm.superSamplingFaktor);
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

subplot(2, 2, 4);
axhist2 = gca;

figure; hold on;
errorPlot = gca;
counter = 0;
for i=1:length(velTime)
    vtrans = [VxRausch(i) 0 0]';
    vrot = [DRotOdoRausch(i) 0]';
    
    vtransc = [vtranscmd(i) 0 0]';
    vrotc = [vrotcmd(i) 0]';
        
    accx = AccXRausch(i);
    omega = DRotIMURausch(i);
    imuTime = velTime(i);
    
    %%
    i1 = velTime(max([i-1 1]));
    i2 = velTime(i); 
    B = intersect(find(posTime>i1),find(posTime<i2));        
    %%
    if ~isempty(B)
        counter = counter + 1;
        if mod(counter,1) == 0
            pos = [XposRausch(B(1)) YposRausch(B(1)) WinkelRausch(B(1))];
            posT = posTime(B(1));
            pf.stepMarker(vtransc, vrotc, vtrans, vrot, accx, omega, imuTime,  pos, posT);
            disp(pos);
        else
            pf.stepIMU(vtransc, vrotc, vtrans, vrot, accx, omega, imuTime);
        end
        
%         dx = pos(1)-pf.CS(1);
%         axes(errorPlot);
%         plot(dx);
%         pf.stepIMU(vtransc, vrotc, vtrans, vrot, accx, omega)
    else
        pf.stepIMU(vtransc, vrotc, vtrans, vrot, accx, omega, imuTime);
    end
    
%     robotPose = [0 Y(i) pi/2];            
%     robot = robot.transform(robotPose(3), robotPose(1:2)');
%     robot = robot.raytrace(env);
%     PCL = robot.generateGlobalPCL();
    %% Scan Matching
    % der Sca wird gegen die Karte auf dem letzen gemachtet    
    % mit der gedachten position
    %dabei der Scan durch die Positionsschätzung der Geschwindigkeits
    %sensoren verrauscht
    T = rot2(npos(3))*PCL';
    T2 = bsxfun(@plus, T, [npos(1) npos(2)]');
    PCLpos = T2';  
    
    startPoses = [...
        0 0 0;                
%         -3*deltaSigmaTrans -3*deltaSigmaTrans -3*deltaSigmaRot;
%         3*deltaSigmaTrans -3*deltaSigmaTrans -3*deltaSigmaRot;
%         -3*deltaSigmaTrans 3*deltaSigmaTrans -3*deltaSigmaRot;
%         3*deltaSigmaTrans 3*deltaSigmaTrans -3*deltaSigmaRot;
%         -3*deltaSigmaTrans -3*deltaSigmaTrans 3*deltaSigmaRot;
%         3*deltaSigmaTrans -3*deltaSigmaTrans 3*deltaSigmaRot;
%         -3*deltaSigmaTrans 3*deltaSigmaTrans 3*deltaSigmaRot;
%         3*deltaSigmaTrans 3*deltaSigmaTrans 3*deltaSigmaRot;                        
];
%     bestscore = 0;
%     cc = [0 0 0];
%     for j=1:size(startPoses)
%         [ccc, score] = sm.matchScan(startPoses(1,:), PCLpos, tm);    
%         if score > bestscore
%             bestscore = score;
%             cc = ccc;
%         end
%     end

        [cc, score] = sm.matchScan(startPoses, PCLpos, tm);    
        T = rot2(cc(3))*PCLpos';
        T2 = bsxfun(@plus, T, [cc(1) cc(2)]');
        PCLcor = T2';     
    

    
    %% Map update
    % updating the map    
    % der erste Scan wird ohne Positionsfehler integriert
    if i==1
        tm = tm.raytraceRayGridPCL(robotPose(1:2), PCL);
    else
        tm = tm.raytraceRayGridPCL(robotPose(1:2), PCLcor);
        
            %% Berechnung des resultierenden Fehler
            % 
%             NPOS  = vertcat(NPOS, abs(npos));
%             e1 = rot2(npos(3))*[cc(1) cc(2)]';
%             err = bsxfun(@plus, e1 , [npos(1) npos(2)]');
%             errr = npos(3) + cc(3);
%             err3 = [err' errr];
%             Err = vertcat(Err, abs(err3));
% 
%             h1 = plot(ax_error, Err(:,1), '-xr');
%             h2 = plot(ax_error, Err(:,2), '-xb');
%             h3 = plot(ax_error, Err(:,3), '-xk');
%             h4 = plot(ax_error, NPOS(:,1), '-or');
%             h5 = plot(ax_error, NPOS(:,2), '-ob');
%             h6 = plot(ax_error, NPOS(:,3), '-ok');            
%             legend(ax_error, [h1 h2 h3 h4 h5 h6], 'x error', 'y error', 'theta error', 'x noise', 'y noise', 'theta noise',  'Location','SouthWest');
            
            
            NPOS  = vertcat(NPOS, norm(npos(1:2)));
            e1 = rot2(npos(3))*[cc(1) cc(2)]';
            err = bsxfun(@plus, e1 , [npos(1) npos(2)]');
            errr = npos(3) + cc(3);
            err3 = [err' errr];
            Err = vertcat(Err, norm(err3(1:2)));
            
%             h1 = plot(ax_error, NPOS, '-xr');
            h1 = plot(ax_error, Err, '-xb');
%             h3 = plot(ax_error, Err(:,3), '-xk');
%             h4 = plot(ax_error, NPOS(:,1), '-or');
%             h5 = plot(ax_error, NPOS(:,2), '-ob');
%             h6 = plot(ax_error, NPOS(:,3), '-ok');            
            legend(ax_error, [h1 ],  'error',   'Location','SouthWest');
    end
    
    pf.plotPaticle(ax, axhist, axhist2);
    pf.plotResPos(axres);
end