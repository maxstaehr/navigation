function [ costs ] = costFunction( f )
%COSTFUNCTION Summary of this function goes here
%   Detailed explanation goes here

costs = 0;

lC = evalin('base', 'lC');
for j=1:length(lC)
    s = lC{j};
    vl = f(1)*s.cmdVelWheelLinks;
    vr = f(2)*s.cmdVelWheelRechts;
    velTime = s.velTime;
    startPos = s.startPos;
    endPos = s.endPosSoll;
    width = evalin('base', 'width');


    vtransreal = (vl + vr)/2;
    vrotreal = (vr - vl)/width;

    P = zeros(size(velTime, 1), 3);
    P(1,:) = startPos;

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
    endPosIst = P(end, :);
    costs = costs + norm(endPosIst(1:2) - endPos(1:2));
end

end

