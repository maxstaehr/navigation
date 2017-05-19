clear all;
pe = PositionEstimator(5, 0, [0 0 0]);


vtrans = [0 0 0]';
vrot = [0 0]';

vtransc = [0 0 0]';
vrotc = [0 0]';

accx = 0;
omega = 0;
imuTime = 1;

%%synchron call
pe.predictstep(vtransc, vrotc, 1);
pe.stepIMU(vtrans, vrot, accx, omega);

pe.predictstep(vtransc, vrotc, 2);
pe.stepIMU(vtrans, vrot, accx, omega);

pe.predictstep(vtransc, vrotc, 3);
pe.stepIMU(vtrans, vrot, accx, omega);

pe.predictstep(vtransc, vrotc, 4);
pe.stepIMU(vtrans, vrot, accx, omega);

pe.predictstep(vtransc, vrotc, 5);
pe.stepIMU(vtrans, vrot, accx, omega);


%%asynchron call
pe.stepIMUAsynchron(vtrans, vrot, accx, omega);
pe.stepSlam([0 0 0], 3);
pe.stepMarker([0 0 0], 3);
pe.stepOF(vtrans, vrot, 3);
pe.recalculate();