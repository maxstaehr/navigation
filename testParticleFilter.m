close all;

maxTime = 5;
dt = 1e-2;
Time = 0:dt:maxTime;
Nt = length(Time);
f =  0.1;
v0 = 1.6;

sigmaAccX = 1e-4;
sigmaVx = 1e-4;
simgaGierrateIMU = 1e-3;
simgaGierrateOdo = 1e-4;


sigmaPosX = 1e-10;
sigmaPosY = 1e-10;
sigmaPosTheta = 1e-10;




DRot =f*sin(Time);
AccX = 0*sin(Time);
V = v0*ones(size(AccX,1),size(AccX,1));
for i=2:length(AccX)
    V(i) = V(i-1)+dt*(AccX(i-1)+AccX(i-1))/2;
end

Winkel = 0*DRot;
Xpos = 0*DRot;
Ypos = 0*DRot;

for i=2:length(DRot)
    Winkel(i) = Winkel(i-1) + DRot(i-1)*dt;
    Xpos(i) = Xpos(i-1)+ cos((Winkel(i-1)))*V(i-1)*dt;
    Ypos(i) = Ypos(i-1)+ sin((Winkel(i-1)))*V(i-1)*dt;
end
figure; hold on;
plot(Time,Winkel, '.k');
plot(Time, DRot, 'xr');
plot(Time, AccX, 'xb');
plot(Time, V, 'xm');
legend('Winkel', 'Gierrate', 'Beschleunigung X', 'Geschwindigkeit X');
figure; hold on; grid on;
xlabel('x');
ylabel('y');
plot(Xpos, Ypos, 'xk');

AccXRausch = AccX + 0*normrnd(0,sigmaAccX, 1, Nt);
DRotIMURausch = DRot +1* normrnd(0,simgaGierrateIMU, 1, Nt);
DRotOdoRausch = DRot + 1*normrnd(0,simgaGierrateOdo, 1, Nt);
VxRausch = V+1* normrnd(0,sigmaVx, 1, Nt);
XposRausch = Xpos +1* normrnd(0,sigmaPosX, 1, Nt);
YposRausch = Ypos +1* normrnd(0,sigmaPosY, 1, Nt);
WinkelRausch = Winkel +1* normrnd(0,sigmaPosTheta, 1, Nt);




% close all;
% figure; hold on;
% plot(Time, AccX, 'xr');
% plot(Time, AccXRausch, 'og');
% legend('Originial', 'Verrauscht');


figure; hold on;
hpos = subplot(2,1,1); hold on; grid on;
plot(XposRausch, YposRausch, 'xr');
plot(hpos, Xpos, Ypos, 'og');
legend('Originial', 'Verrauscht');
hparticle = subplot(2,1,2); hold on; grid on;

%%definiere State space
% x
% y 
% theta
% vtrans 
% vrot
N = 1e3;



Var = 5*([sigmaPosX sigmaPosX sigmaPosTheta sigmaVx simgaGierrateOdo])'; %define the variance of the initial esimate
Var =  (Var .* Var);
x = [Xpos(1) Ypos(1) Winkel(1) V(1) DRot(1)]';
x_P = []; % define the vector of particles

% make the randomly generated particles from the initial prior gaussian distribution
x_P = zeros(length(x), N);
for i = 1:N
    for j=1:length(x)
        x_P(j,i) = x(j) + sqrt(Var(j)) * randn;
    end
end
% close all;
% figure; hold on; grid on;
% plot(x_P(1,:), x_P(2,:), 'xr');

x_out = [x];  %the actual output vector for measurement values.
x_est = [x]; % time by time output of the particle filters estimate
x_est_out = []; % the vector of particle filter estimates.
%%for all time steps
for j=1:Nt
    x_P_update = nextState(x_P, dt);
%     plot(hpos, x_P_update(1,:), x_P_update(2,:), 'ob')
    
    %%update the particles based upon the odometrie data
    Z_odo = generateMeasurementODO(x_P_update);
    Z_IMU = generateMeasurementIMU(x_P_update, x_P, dt );
    
    P_w_odo = zeros(2, N);    
    %%Update Gewichtung anhand von Messung
    x_R = sigmaVx^2;
    z = VxRausch(j);
    z_update = Z_odo(1,:);
    P = zeros(1,N);
    for i=1:N
        v = (1/sqrt(2*pi*x_R)) * exp(-(z - z_update(i))^2/(2*x_R));
        P_w_odo(1,i)= (1/sqrt(2*pi*x_R)) * exp(-(z - z_update(i))^2/(2*x_R));
    end
%     if ~isempty(find(isnan(P)))
%        disp('bullshit');
%     end    
%     sumP = sum(P);
%     if sumP > 0
%         P_w_odo(1,:) = P_w_odo(1,:)./sumP;
%     end
    
    
    
    x_R = simgaGierrateOdo^2;
    z = DRotOdoRausch(j);
    z_update = Z_odo(2,:);
    for i=1:N
        P_w_odo(2,i) = (1/sqrt(2*pi*x_R)) * exp(-(z - z_update(i))^2/(2*x_R));
    end
%     if sum(P_w_odo(2,:) > 0
%         P_w_odo(2,:) = P_w_odo(2,:)./sum(P_w_odo(2,:));
%     end
    
%     close all; figure; hold on;
%     plot(z, 'og','markersize',25);
%     plot(z_update, 'or');
    
    P_w_imu = zeros(2, N);
    %%Update Gewichtung anhand von Messung
    x_R = sigmaAccX^2;
    z = AccXRausch(j);
    z_update = Z_IMU(1,:);
    for i=1:N
        P_w_imu(1,i) = (1/sqrt(2*pi*x_R)) * exp(-(z - z_update(i))^2/(2*x_R));
    end    
%     P_w_imu(1,:) = P_w_imu(1,:)./sum(P_w_imu(1,:));    
    
    x_R = simgaGierrateIMU^2;
    z = DRotIMURausch(j);
    z_update = Z_IMU(2,:);
    for i=1:N
        P_w_imu(2,i) = (1/sqrt(2*pi*x_R)) * exp(-(z - z_update(i))^2/(2*x_R));
    end
%     P_w_imu(2,:) = P_w_imu(2,:)./sum(P_w_imu(2,:));    
    

    P1 = normalizeP(P_w_odo(1,:));
    P2 = normalizeP(P_w_odo(2,:));
    P3 = normalizeP( P_w_imu(2,:));
    P4 = normalizeP( P_w_imu(2,:));    
    P_w = P1 + 100*P2 +P3+ 100*P4;
    %P_w =  P_w_odo(1,:) + P_w_odo(2,:) + P_w_imu(2,:);
    % Normalize to form a probability distribution (i.e. sum to 1).
    P_w = P_w./sum(P_w);
   [C,I] = max(P_w_odo(2,:));
   index = I(1) ;
    %weighted estimate
    x_est = zeros(5,1);
    for i = 1 : N
        x_est = x_est + P_w(i) *x_P_update(:,i);
    end
%     x_est = x_P_update(:,index);
     %plot(x_est(5), 'or');
%      plot(x_est(5), 'om','markersize',25);
%     plot(x_est(1),x_est(2),'.r','markersize',50) 
%     C = bsxfun(@minus,x_P_update,x_est); 
%     covarianz = cov(x_P_update');

    Diff = bsxfun(@minus, x_P_update, x_est);
    
    cla(hparticle)
%     plot(hparticle, x_P_update(5,:), 'xr');
%     plot(hparticle, x_est(5,:), 'og', 'MarkerSize', 50);
%     plot(hparticle, DRot(j), 'om', 'MarkerSize', 50);
    varVector = var(Diff,ones(1,size(Diff,2)),2);
    %varVector = max(abs(Diff),[],  2);
    varVector = max([varVector 1e-1*[1e-6 1e-6 1e-6 1e-6 1e-6]'],[],  2);
    %disp([varVector(5) Nunique]);
    disp([Nunique]);
%    
%     varVector = var(x_Var,ones(1,size(x_Var,2)),2);

    Index = zeros(1,N);
    for i = 1 : N
        Index(i) = find(rand <= cumsum(P_w),1);
    end
    UniqueIndex  = unique(Index);
    Nunique = length(UniqueIndex);
    x_P1 = x_P_update(:,UniqueIndex);   
   Plot1  = bsxfun(@minus, x_P1, x_est);
   
   x_P2 = bsxfun(@times, randn(5,N-Nunique), sqrt(varVector));   
   Plot2 = x_P2;      
   x_P2 = bsxfun(@plus, x_P2, x_est);
   if ~isempty(find(isnan(x_P2)))
       disp('bullshit');
   end
   x_P = horzcat(x_P1 , x_P2);
   
    
    plot(hparticle, Plot1(1,:), Plot1(2,:), 'xm');
    plot(hparticle, Plot2(1,:), Plot2(2,:), 'xr');
    xlim(hparticle, 2e-3*[-1 1])
    ylim(hparticle, 2e-3*[-1 1])
    xlabel('x [m]');
    ylabel('y [m]');
    legend('particles from previous state', 'new generated particles');
%    x_P =  mvnrnd(x_est', covarianz, N);
%    x_P = x_P';
%    x_P =  bsxfun(@plus,x_P,x_est); 
   


    
    plot(hpos, x_P(1,:), x_P(2,:), '*m');      
    plot(hpos, x_est(1,:), x_est(2,:), 'or', 'MarkerSize', 25); 
    legend(hpos, 'ground truth position','particles along route', 'estimated positions');
    x_est_out = [x_est_out x_est];
%    pause(0.001);
end


figure; hold on; grid on; 
ylabel('y');
plot(Xpos, Ypos, 'xk');
plot(x_est_out(1,:), x_est_out(2,:), 'og');
legend('Ground Truth', 'Est');


DRotIMURausch = DRot +1* normrnd(0,simgaGierrateIMU, 1, Nt);
DRotOdoRausch = DRot + 1*normrnd(0,simgaGierrateOdo, 1, Nt);
figure; hold on; grid on;
title('error rotational speed');
plot(Time, DRotIMURausch-DRot , 'xb');
plot(Time,DRotOdoRausch-DRot, '.k');
plot(Time, x_est_out(5,:)-DRot, 'm');
xlabel('time [sec]');
ylabel('error [rad/s]');
legend( 'Error IMU', 'Error Odo', 'Error Est');

figure; hold on;
title('error translational speed');
plot(Time,VxRausch-V, '.k');
plot(Time, x_est_out(4,:)-V, 'm');
xlabel('time [sec]');
ylabel('error [m/s]');
legend( 'Error IMU', 'Error Odo', 'Error Est');

