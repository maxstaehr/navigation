figure; hold on;
pos = GP(1,:);
theta = pos(3);

P = zeros(size(velTime, 1), 3);
P(1,:) = GP(1,:);
for i=2:length(velTime)
    dt = velTime(i) - velTime(i-1);
    xpos = P(i-1,1);
    ypos = P(i-1, 2);
    theta = P(i-1, 3);
    
    drot = (vrotreal(i) + vrotreal(i-1))/2*dt;
    dtrans = (vtransreal(i) +  vtransreal(i-1))/2* dt;
    P(i,1) = cos(theta + drot)*dtrans + xpos;
    P(i, 2) = sin(theta + drot)*dtrans + ypos;
    P(i, 3) = theta + drot;  
    
end
plot(TR(:,1), TR(:,2), 'og');
plot(P(:,1), P(:,2), 'xr');