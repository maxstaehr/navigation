syms theta x y vtrans vrot dt
fx = cos(theta +vrot*dt)*vtrans + x;
fy = sin(theta +vrot*dt)*vtrans + y;
ftheta = theta+vrot*dt;
fvtrans = vtrans;
fvrot = vrot;
jacobian([fx, fy, ftheta, fvtrans, fvrot], [x, y, theta, vtrans, vrot])