pf = PF.PF();
vtrans = [1 0 0 ]';
vrot = [0 0 ]';
pf.predict(vtrans, vrot);
figure; hold on; grid on;
ax = gca;
for i=1:100
    pf.step(vtrans, vrot);
    pf.plotPaticle(ax);
end