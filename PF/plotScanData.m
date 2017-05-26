A = linspace(-pi, pi/2, 541);
Xf = cos(A);
Yf = sin(A);
close all;figure; hold on; axis equal; grid on;
data = laserRanges;
h = plot(nan, nan, 'x');
%% Set up the movie.
% writerObj = VideoWriter('out.avi'); % Name it.
% writerObj.FrameRate = 1/12.5; % How many frames per second.
% open(writerObj); 
slice = 100;
for i=1:length(data)
    range = fliplr(data{i});
    range(range > 25) = nan;
    range = range(1:541);
    range(1:slice) = nan;
    range(end-slice:end) = nan;
    
    X = Xf .* range;
    Y = Yf .* range;
    set(h, 'XData', X);
    set(h, 'YData', Y);
    xlim([-30 30]);
    ylim([-30 30]);
    pause(1/12.5);
    disp(i);
    
%      frame = getframe(gcf); % 'gcf' can handle if you zoom in to take a movie.
%     writeVideo(writerObj, frame);
    
end
close(writerObj); % Saves the movie.