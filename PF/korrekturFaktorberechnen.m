load('input.mat');
startPos = GP(1,:);
endPos = GP(end,:);
pos =  GP(1,:);


P = zeros(size(velTime, 1), 3);
P(1,:) = GP(1,:);

for i=2:length(velTime)
    dt = velTime(i) - velTime(i-1);

    xpos = P(i-1,1);
    ypos = P(i-1, 2);
    theta = P(i-1, 3);
    
    drot = ((vrotreal(i) + vrotreal(i-1))/2)*dt;
    dtrans = ((vtransreal(i) +  vtransreal(i-1))/2)* dt;
    P(i,1) = cos(theta + drot)*dtrans + xpos;
    P(i, 2) = sin(theta + drot)*dtrans + ypos;
    P(i, 3) = theta + drot;  
    
   
end
endPosSoll = [P(end,1) P(end,2)];
width = .525;
cmdVelRotLinks = -0.5*vrotreal*width;
cmdVelRotRechts = 0.5*vrotreal*width;
cmdVelWheelLinks = vtransreal + cmdVelRotLinks;
cmdVelWheelRechts = vtransreal + cmdVelRotRechts;



close all;figure; hold on; grid on;

plot(TR(:,1), TR(:,2));
plot(GP(:,1), GP(:,2),'og');
k = 27;
meas = GP(:,1:2);
Z = linkage(meas,'ward','euclidean');
c = cluster(Z,'maxclust',k);
lC = {};
startIndex = c(1);
ci = 0;
for i=1:length(c)-1
    if c(i) ~= c(i+1)
        s = struct();
        s.startPos =  GP(i,:);
        s.endPosSoll =  GP(i+1,:);
        Index = intersect (find(velTime>posTime(i)), find(velTime<= posTime(i+1)));
        s.cmdVelWheelLinks = cmdVelWheelLinks(Index);
        s.cmdVelWheelRechts = cmdVelWheelRechts(Index);
        s.velTime = velTime(Index);        
        lC{end+1} = s;        
        ci = ci +1;
        if ci > 6
            break;
        end
    end
    
end

for i = 1:k
    index = find(c==i);
    plot(GP(index,1), GP(index,2),'xr');
end

% cluster(Z,'maxclust',4);
% [idx,C]  = kmeans(GP(:,1:2),k);
% plot(C(:,1),C(:,2),'kx',...
%      'MarkerSize',15,'LineWidth',3)




% offsetEnde = 77;
% line([GP(end-offsetEnde,1)  P(end,1)], [GP(end-offsetEnde,2)  P(end,2)]);
% Index = intersect (find(velTime>posTime(1)), find(velTime<= posTime(end-offsetEnde)));
% cmdVelWheelLinks = cmdVelWheelLinks(Index);
% cmdVelWheelRechts = cmdVelWheelRechts(Index);
% velTime = velTime(Index);
% plot(P(Index,1), P(Index,2),'xr');

options = optimset('Display','iter');
options = optimset(options,'TolFun', 1e-8);
% x = fminsearchbnd(@costFunction,[1 1 1],0.95*[1 1 1], 1.05*[1 1 1]) ;
% x = fminbnd(@costFunction,0.95*[1 1], 1.05*[1 1]);
% disp(x);

options = saoptimset;
%% Modify options setting
x0 = [1 1];
lb = 0*x0;
ub = 2*x0;
options = saoptimset(options,'ObjectiveLimit', 1e-8);
options = saoptimset(options,'MaxFunEvals', 3000000);
options = saoptimset(options,'Display', 'iter');
options = saoptimset(options,'HybridInterval', 'end');
[x,fval,exitflag,output] = ...
simulannealbnd(@costFunction,x0,lb,ub,options);
fprintf('\n\nFaktor links: %.9f\tFaktor rechts: %.9f\n\n', xc(1), xc(2));


vl = x(1)*evalin('base', 'cmdVelWheelLinks');
vr = x(2)*evalin('base', 'cmdVelWheelRechts');
velTime = evalin('base', 'velTime');
startPos = evalin('base', 'startPos');
endPos = evalin('base', 'endPos');
width = evalin('base', 'width');


vtransreal = (vl + vr)/2;
vrotreal = (vr - vl)/width;

P2 = zeros(size(velTime, 1), 3);
P2(1,:) = startPos;

for i=2:length(velTime)
    dt = velTime(i) - velTime(i-1);

    xpos = P2(i-1,1);
    ypos = P2(i-1, 2);
    theta = P2(i-1, 3);
    
    drot = ((vrotreal(i) + vrotreal(i-1))/2)*dt;
    dtrans = ((vtransreal(i) +  vtransreal(i-1))/2)* dt;
    P2(i,1) = cos(theta + drot)*dtrans + xpos;
    P2(i, 2) = sin(theta + drot)*dtrans + ypos;
    P2(i, 3) = theta + drot;  
    
   
end
plot(P2(:,1), P2(:,2),'xm');


