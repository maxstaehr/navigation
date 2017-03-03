%%zuerst einen Aufbau machen um den das System rotiert wird
close all; figure(1); hold on; grid on;  axis equal
% f_3Dwf('k');


%
SensorPunkt = [-6 -1 0];
f =  0.5;
f2 = 0;
v0 = 1;
R = rpy2r(0, 0, 0);
H = [R SensorPunkt'; 0 0 0 1];
% f_3Dframe(H,'g',1,'_{sensor}');

N = 1e3;
maxTime = 30;
dt = maxTime/N;
Time = 0:dt:maxTime;


DRot =f*cos(Time);
AccX = f2*sin(Time);
V = v0*ones(size(AccX,1),size(AccX,1));
for i=2:length(AccX)
    V(i) = V(i-1)+dt*(AccX(i-1)+AccX(i-1))/2;
end



Winkel = 0*DRot;
Xpos = 0*DRot;
Ypos = 0*DRot;

for i=2:length(DRot)
    Winkel(i) = trapz(Time(1:i),DRot(1:i)*dt);
    Xpos(i) = Xpos(i-1)+ cos((Winkel(i)+ Winkel(i-1))/2)*dt;
    Ypos(i) = Ypos(i-1)+ sin((Winkel(i)+ Winkel(i-1))/2)*dt;
end
plot(Time,Winkel, '.k');
plot(Time, DRot, 'xr');
plot(Time, AccX, 'xb');
plot(Time, V, 'xm');
legend('Winkel', 'Gierrate', 'Beschleunigung X', 'Geschwindigkeit X');
figure; hold on; grid on;  axis equal
xlabel('x');
ylabel('y');
% plot(Xpos, Ypos, 'xk');


SensorPunkt = SensorPunkt';
L = {};
M = [];
ASensor= [];
ARoboter= [];
Omega = [];
C = [];
c = 0;
for i=1:length(Time)    
    
    v = V(i);
    gierrate = DRot(i);
    accx = AccX(i);
    HWorld2Roboter = [rpy2r(0, 0, Winkel(i)) [Xpos(i) Ypos(i) 0]'; 0 0 0 1];
%     test = HWorld2Roboter*[0 1 0 1]';
    %H1 = [eye(3) [Xpos(i) Ypos(i) 0]'; 0 0 0 1];
    %HWorld2Roboter = H1*H2;
    HRoboter2Sensor = H;
    HWorld2Sensor = HWorld2Roboter * HRoboter2Sensor;
    
        
    %%
    %Berechung des Mittelpunkt um den sich der Roboter dreht
    if DRot(i) > 0
        m = [0 1 0];
        f = sqrt((v^2)/(gierrate^2));
    elseif DRot(i) < 0
        m = [0 -1 0];
        f = sqrt((v^2)/(gierrate^2));
    else
        m = [0 0 0];
        f = 0;
        
    end
     
    
     VMittelpunktRoboter = HWorld2Roboter(1:3,4)';
     
                               
     %%resultierenden Vektor berechnen
     VDrehpunkt = f*m;
     VSensor = HRoboter2Sensor(1:3,4)';            
     VMittelPunktZuSensor = (VSensor-VDrehpunkt);
     
    %Plotten der Ergebnisse
%     f_3Dframe(HWorld2Roboter,'g',1,'_{robot}');
%     f_3Dframe(HWorld2Sensor,'g',1,'_{sensor}');
%    mArrow3([0 0 0], VDrehpunkt,'color', 'r');
%    mArrow3([0 0 0], VSensor, 'color', 'b');
%     mArrow3(VDrehpunkt, VDrehpunkt+VMittelPunktZuSensor, 'color', 'g');
    

%Berechnung resultierende Beschleunigung Sensor durch Kreisfahrt
    radiusZuSensor = norm(VMittelPunktZuSensor);
    aBetragSensor = (gierrate^2)*radiusZuSensor;
    VMittelPunktZuSensorNormiert = VMittelPunktZuSensor/norm(radiusZuSensor);    
    VresultierendeBeschleunigungSensor = aBetragSensor*VMittelPunktZuSensorNormiert + accx * [1 0 0];
    %mArrow3(VSensor, VSensor+VresultierendeBeschleunigungSensor, 'color', 'm');
    
    ASensor = vertcat(ASensor, VresultierendeBeschleunigungSensor);
%Berechnung resultierende Beschleunigung Roboter
    
    radiusZuMittelpunkt = norm(VDrehpunkt);
    aBetragRoboter = (gierrate^2)*radiusZuMittelpunkt;
    VMittelPunktZuRoboterNormiert = -VDrehpunkt/norm(VDrehpunkt);
    VresultierendeBeschleunigungRoboter = aBetragRoboter*VMittelPunktZuRoboterNormiert + accx * [1 0 0];
    %mArrow3([0 0 0],VresultierendeBeschleunigungRoboter, 'color', 'm');
    ARoboter = vertcat(ARoboter, VresultierendeBeschleunigungRoboter);
    
    Omega = vertcat(Omega, DRot(i));
    
%%backwars    
    x = [ 2.000000000000000  0];
    VZentrifugal = VresultierendeBeschleunigungSensor- accx * [1 0 0];
    VZentripedalRichtung  = -VZentrifugal/norm(VZentrifugal);
    radiusSensor = norm(VZentrifugal)/(gierrate^2);
    VDrehpunkt = radiusSensor*VZentripedalRichtung;    
    VSensorMittelPunkt = [x(1) x(2) 0];
    %mArrow3(-VSensorMittelPunkt,-VSensorMittelPunkt+VDrehpunkt, 'color', 'm');
    VDrehPunktMittelPunkt = VSensorMittelPunkt-VDrehpunkt;
%     mArrow3(-VSensorMittelPunkt+VDrehpunkt,-VSensorMittelPunkt+VDrehpunkt+VDrehPunktMittelPunkt, 'color', 'm');
    
    radiusZuMittelpunkt = norm(VDrehPunktMittelPunkt);
    aBetragRoboter = (gierrate^2)*radiusZuMittelpunkt;
    VMittelPunktZuRoboterNormiert = VDrehPunktMittelPunkt/norm(VDrehPunktMittelPunkt);
%     mArrow3(-VSensorMittelPunkt+VDrehpunkt,-VSensorMittelPunkt+VDrehpunkt+VDrehPunktMittelPunkt, 'color', 'm');
    
    VresultierendeBeschleunigungRoboter = aBetragRoboter*VMittelPunktZuRoboterNormiert;
    c1 = VresultierendeBeschleunigungRoboter(1);
    c2 = abs(VresultierendeBeschleunigungRoboter(2))-((gierrate^2)*radiusZuMittelpunkt) ;
    
    C = vertcat(C, c1+c2);
end
figure; hold on; grid on;
plot(C);
figure; hold on; grid on;
plot(ASensor(:,1), '*g');
plot(ASensor(:,2), 'xr');
plot(ARoboter(:,2), 'ob');
legend('Sensor X', 'Sensor Y', 'Roboter');
save('testData.mat', 'Omega', 'ASensor');

%plot(M(:,1), M(:,2), 'mx');
% Xpos = Xpos - Xpos(1);
% Ypos = Ypos - Ypos(1);
% Zpos = Zpos - Zpos(1);



