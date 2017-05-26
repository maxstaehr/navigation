%clear all;
close all; 
load('np_vector.mat');

%Tiefpassfilterung 
f_tp = 0.5;
fs = 200;
[b, a] = butter(5,f_tp/(0.5*fs),'low'); 



% accx =  filter(b,a,accx);
% omega =  filter(b,a,omega);
% plot(omega, '-b')

w = 0.525;
offset = min(velTime);
duration = max(velTime)-min(velTime);
time = 0:0.01:duration;
vlReal = interp1(velTime, vlReal, time+offset);
vrReal = interp1(velTime, vrReal, time+offset);
vlDesLim = interp1(velTime, vlDesLim, time+offset);
vrDesLim = interp1(velTime, vrDesLim, time+offset);
velTime = time+offset;
%plot(time, vlReal,time, vrReal);
%plot(time, vlDesLim,time, vlReal);
%  plot(time, vrDesLim,time, vrReal);


[ vtransreal, vrotreal ] = berechnetVtranVrot(vlReal, vrReal, w);
[ vtranscmd, vrotcmd ] = berechnetVtranVrot(vlDesLim, vrDesLim, w);

% f = [1.004822203474123   1.017038207315438];
%  [ vtransreal, vrotreal ] = korrigiereV( vtransreal, vrotreal , f);


 pos = [Globalx(1) Globaly(1) Globaltheta(1)];
 pe = PositionEstimator(150,pos);
 endIndex = floor(length(velTime)/6);
 AccX = interp1(imuTime, accx, velTime);
 Omega = interp1(imuTime, omega, velTime);
 
 pos = [Globalx(1) Globaly(1) Globaltheta(1)];
P = zeros( endIndex, 3);
P(1,:) = pos(1,:);


for i=2:endIndex
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
    
   
end

 


P2 = zeros( endIndex, 3);
for i=2:endIndex
    %% current values
    % DESCRIPTIVE TEXT
    
    
    vtrans = [vtransreal(i) 0 0]';
    vrot = [vrotreal(i) 0]';
    ax = AccX(i);
    o = Omega(i);
    imuTime = velTime(i);
    
   
    
    vtransc = [vtranscmd(i) 0 0]';
    vrotc = [vrotcmd(i) 0]';
    
    
        

    %%check for update
    i1 = velTime(max([i-1 1]));
    i2 = velTime(i); 
    
    
    %% check for marker update
    B = intersect(find(GlobalTime>i1),find(GlobalTime<=i2));        
    updateMarker = ~isempty(B);    

    
    pe.predictstep(vtransc, vrotc, imuTime);
    pe.stepIMUVel(vtrans, vrot, ax, o);
    if updateMarker           
        pos = [Globalx(B(1)) Globaly(B(1)) Globaltheta(B(1))];
        posT = GlobalTime(B(end));                       
        pe.stepMarker(pos, posT);                   
    end
    pe.recalculate();
        
    P2(i,:) =  pe.currentState.CS(1:3)';
    V(:, i) = pe.currentState.varianz;    
    pfStates{i} = pe.currentState.PRED(1:2,:);
    

end




