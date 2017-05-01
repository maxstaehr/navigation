
pf = PF.PF();
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
subplot(2, 2, 4);
axhist2 = gca;
for i=1:length(x)
    pf.step(vtrans, vrot, accx, omega, [x(i) 0 0]);
    pf.plotPaticle(ax, axhist, axhist2);
end