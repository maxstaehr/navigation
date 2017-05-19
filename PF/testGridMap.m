vx = 0.02;
vy = 0.02;
nx = 100;
ny = 100;
d = 2;
range = 0:0.01:d;
theta = pi/4;
ct = cos(theta);
st = sin(theta);

PX = ct * range;
PY = st*range;
IX =  floor(PX ./ 0.02);
IY =  floor(PY ./ 0.02);
%%
d = 0.1;
xmax = 10;
xmin = -10;
ymax = 10;
ymin = -10;
width = xmax-xmin;
height = ymax-ymin;
nx = floor(width/d)+1;
ny = floor(height/d)+1;


xvoxelwidth = (xmax -xmin)/(nx-1);
yvoxelwidth = (ymax -ymin)/(ny-1);

[X, Y] = meshgrid(xmin: xvoxelwidth:xmax, ymin:yvoxelwidth:ymax);
ws_x = reshape(X,1,numel(X));
ws_y = reshape(Y,1,numel(Y));
ws = 0*ws_y;

A = [IX; IY]';
C = unique(A,'rows');
for i=1:size(C,1)
    [xid, yid] = calcIndexInWS( xmax, xmin, ymax, ymin, C(i,:), nx, ny);
    id = calcResIndexWS(xid, yid, ny);    
    
    ws(id) = 1;
    
end

C = [];
for i=1:length(ws)
    if ws(i)
        C = vertcat(C, [ws_x(i) ws_y(i)]);
    end
end

close all; figure; hold on; axis equal;
plot(C(:,1), C(:,2), '.r');

