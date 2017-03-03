close all;
p = ParticleFilter();

figure; hold on; grid on;
ax = gca;
p.predict([1 0 0], [0 0 0]);
p.step();
p.plotPaticle(ax);
p.predict([1 0 0], [0 0 0]);
p.step();
p.plotPaticle(ax);
p.predict([1 0 0], [0 0 0]);
p.step();
p.plotPaticle(ax);