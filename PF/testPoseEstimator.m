
pe = PositionEstimator(5, [0 0 0]);


vtrans = [1 0 0]';
vrot = [0 0]';

distL = 1;
distR = 1;

vtransc = [1 0 0]';
vrotc = [0 0]';

accx = 0;
omega = 0;
imuTime = 1;

%%synchron call
vtransc = [0 0 0]';
vrotc = [0 0]';
vtrans = [0 0 0]';
vrot = [0 0]';
pe.predictstep(vtransc, vrotc, 0);
pe.stepIMUDist(0, 0, accx, omega, 0);
pe.recalculate();
disp(pe.currentState.CS(1:3));

vtransc = [1 0 0]';
vrotc = [0 0]';
vtrans = [1 0 0]';
vrot = [0 0]';
pe.predictstep(vtransc, vrotc, 1);
pe.stepIMUDist(0.5, 0.5, accx, omega, 1);
pe.recalculate();
disp(pe.currentState.CS(1:3));

pe.predictstep(vtransc, vrotc, 2);
pe.stepIMUDist(1.5, 15, accx, omega, 2);
pe.recalculate();
disp(pe.currentState.CS(1:3));

pe.predictstep(vtransc, vrotc, 3);
pe.stepIMUDist(2.5, 2.5, accx, omega, 3);
pe.recalculate();
disp(pe.currentState.CS(1:3));

pe.predictstep(vtransc, vrotc, 4);
pe.stepIMUDist(3.5, 3.5, accx, omega, 4);
pe.recalculate();
disp(pe.currentState.CS(1:3));

pe.predictstep(vtransc, vrotc, 5);
pe.stepIMUDist(4.5, 4.5, accx, omega, 5);
pe.recalculate();
disp(pe.currentState.CS(1:3));


% %%asynchron call
% pe.stepIMUAsynchron(vtrans, vrot, accx, omega);
% pe.stepSlam([3 0 0], 3.1);
% pe.stepMarker([3 0 0], 3.1);
% pe.stepOF(vtrans, vrot, 3.1);
% pe.recalculate();
% disp(pe.currentState.CS(1:3));

