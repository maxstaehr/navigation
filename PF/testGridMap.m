close all;
vx = 0.02;
vy = 0.02;
nx = 10;
ny = 10;

l = 2;
range = 0:0.5:l;
theta = pi/4;
ct = cos(theta);
st = sin(theta);

PX = ct * range;
PY = st*range;

d = 1;
%%
xmax = 5;
xmin = 0;
ymax = 5;
ymin = 0;
width = xmax-xmin;
height = ymax-ymin;
nx = floor(width/d)+1;
ny = floor(height/d)+1;


xvoxelwidth = (xmax -xmin)/(nx-1);
yvoxelwidth = (ymax -ymin)/(ny-1);

IX =  floor((PX - xmin) ./ xvoxelwidth)+1;
IY =  floor((PY - ymin) ./ yvoxelwidth)+1;

[X, Y] = meshgrid(xmin: xvoxelwidth:xmax, ymin:yvoxelwidth:ymax);
ws_x = reshape(X,1,numel(X));
ws_y = reshape(Y,1,numel(Y));
ws = 0*ws_y;

A = [IX; IY]';
C = unique(A,'rows');
for i=1:size(C,1)
    %[xid, yid] = calcIndexInWS( xmax, xmin, ymax, ymin, [ws_x(C(i,1)) ws_y(C(i,2))] , nx, ny);
    id = calcResIndexWS(C(i,1), C(i,2), ny);    
    
    ws(id) = 1;
    
end

D= [];
for i=1:length(ws)
    if ws(i)
        D = vertcat(D, [ws_x(i) ws_y(i)]);
    end
end

close all; figure; hold on; axis equal; grid on;
xlim([xmin xmax]);
ylim([ymin ymax]);
xt = xmin: xvoxelwidth:xmax;
yt = ymin: yvoxelwidth:ymax;
X = D(:,1)'+xvoxelwidth/2;
Y = D(:,2)'+yvoxelwidth/2;
plot(X,Y, '.r');
f = gca();
set(f, 'XTick', xt);
set(f, 'YTick', yt);


