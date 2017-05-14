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
    pf.plotPaticle(ax, axhist, axhist2);
    pf.plotResPos(axres);
end