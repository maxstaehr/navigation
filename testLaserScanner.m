
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

% LL{end+1} = bsxfun(@plus, [5 -10], L);
% LL{end+1} = bsxfun(@plus, [5 -5], L);
% LL{end+1} = bsxfun(@plus, [5 0], L);
% LL{end+1} = bsxfun(@plus, [5 5], L);
% LL{end+1} = bsxfun(@plus, [5 10], L);
% LL{end+1} = bsxfun(@plus, [5 15], L);
% LL{end+1} = bsxfun(@plus, [5 20], L);
% LL{end+1} = bsxfun(@plus, [5 25], L);
    
env = Environment(LL);











close all;subplot(2,2,1); hold on; grid on; axis equal;
ax_env = gca;
env.plot(ax_env);
subplot(2,2,2); hold on; grid on; axis equal;
ax_map = gca;
subplot(2,2,3); hold on; grid on; axis equal;
ax_prob = gca;
subplot(2,2,4); hold on; grid on; 
ax_error = gca;
% figure; hold on; grid on; axis equal;
% ax_prob = gca;

%% Grid map
% grid map
% tm = GridMap(-50, 20,-20, 35, 0.5, deg2rad(0.5));
tm = GridMap(-50, 20,-20, 35, 0.1, deg2rad(0.5));
robot = Robot(tm.superSamplingFaktor);


tmax = 15;
tdelta = 0.05;
MatchingZyklen = 20;
T = 0:MatchingZyklen*tdelta:tmax;

maxVelTrans = 2.0;
maxVelRot = 2*pi/10;
errorFaktorTrans = 0.3;
errorFaktorRot = 0.3;

deltaSigmaTrans = errorFaktorTrans*maxVelTrans/6;
deltaSigmaRot = errorFaktorRot*maxVelRot/6;

nx = ceil(6*deltaSigmaTrans/tm.xvoxelwidth);
ny = ceil(6*deltaSigmaTrans/tm.xvoxelwidth);
sm = ScanMatcher([6*deltaSigmaTrans 6*deltaSigmaTrans 6*deltaSigmaRot],nx, ny);

Y = T*maxVelTrans-10;

S = struct();
S.fA = robot.frontScanner.A;
S.rA = robot.rearScanner.A;

D = cell(1,length(Y));
N = [deltaSigmaTrans*randn(length(Y), 1) deltaSigmaRot*randn(length(Y), 1)];

Err = [];
NPOS = [];
for i=1:length(Y)
    
    cla(ax_map);
    cla(ax_prob);
    
    %% Noise Berechnung
    % es wird der Positionsfehler anhand der rauschenden 
    % translatorischen und rotatorischen Geschwindigkeit berechnet
    n = N(i,:);
    dtheta = n(2)*tdelta*MatchingZyklen;
    dtrans = n(1)*tdelta*MatchingZyklen;
    dx = cos(pi/2+dtheta)*dtrans;
    dy = sin(pi/2+dtheta)*dtrans;
    npos = [dx dy dtheta];
    
%     npos = [dx dy 0];
    
%% Raytracing
% raytraces the environment    
    

    robotPose = [0 Y(i) pi/2];            
    robot = robot.transform(robotPose(3), robotPose(1:2)');
    robot = robot.raytrace(env);
    PCL = robot.generateGlobalPCL();
    

    

    %% Scan Matching
    % der Sca wird gegen die Karte auf dem letzen gemachtet    
    % mit der gedachten position
    %dabei der Scan durch die Positionsschätzung der Geschwindigkeits
    %sensoren verrauscht
    T = rot2(npos(3))*PCL';
    T2 = bsxfun(@plus, T, [npos(1) npos(2)]');
    PCLpos = T2';  
    
    
    cc = sm.matchScan([0 0 0], PCLpos, tm);    
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
    %% SECTION TITLE
    % plotting result
%     cla(ax_env); hold on;
%     h1 = env.plot(ax_env);
%     h2 = plot(ax_env, PCL(:,1), PCL(:,2), '.m');    
%     h3 = plot(ax_env, PCLpos(:, 1), PCLpos(:,2), 'xk');
%     h4 = plot(ax_env, PCLcor(:, 1), PCLcor(:,2), 'og');    
%     [h5, h6] = tm.plotWeightedVoxel(ax_env);
%     legend(ax_env, [h1 h2 h3 h4 h5 h6], 'Ground truth', 'True Points', 'After Estimate', 'After Correction', 'Weighted','Center','Location','SouthWest');    
%     
%     
% %     tm.plotOccupancy(ax_map);
%     
%     tm.plotProbabilityMap(ax_prob);


    
    
    

    pause(1);
    disp(i);
end
%save('laserScanData.mat', 'D', 'S');


