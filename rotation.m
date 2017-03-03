roll = 0;
pitch = pi/4;
yaw = 0;

vDrehrate = [0 0 1]';
n = [0 0 -1];

R = rpy2r(roll, pitch, yaw);
wx = vDrehrate(1);
wy = vDrehrate(2);
wz = vDrehrate(3);
Drehmatrix = [0 0 wy; wz 0 0; 0 wx 0];
resultiertendeDrehmatrix =  R *vDrehrate;
