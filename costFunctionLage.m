function [ c ] = costFunctionLage( x )
%COSTFUNCTIONLAGE Optimiert die Lage der Sensors

Omega = evalin('base', 'Omega');
ASensor = evalin('base', 'ASensor');
c = 0;

for i=1:length(Omega)
    VresultierendeBeschleunigungSensor = ASensor(i,:);
    gierrate = Omega(i);
    
    VZentrifugal = VresultierendeBeschleunigungSensor;
    VZentripedalRichtung  = -VZentrifugal/norm(VZentrifugal);
    radiusSensor = norm(VZentrifugal)/(gierrate^2);
    VDrehpunkt = radiusSensor*VZentripedalRichtung;    
    VSensorMittelPunkt = [x(1) 0 0];
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
    c = c +c1^2 + c2^2;
    
    
end

end

