% figure; hold on; axis equal; grid on;
% A = linspace(-pi, pi/2, 541);
% Xf = cos(A);
% Yf = sin(A);
% close all;figure; hold on; axis equal; grid on;
% h = plot(nan, nan, 'x');
% for i=1:length(data)
%     range = fliplr(data{i});
%     range(range > 25) = nan;
%     range = range(1:541);
%     
%     X = Xf .* range;
%     Y = Yf .* range;
%     set(h, 'XData', X);
%     set(h, 'YData', Y);
%     xlim([-30 30]);
%     ylim([-30 30]);
%     pause(1/12.5);
%     disp(i);
%     
end
% close all;
% time = velTime - min(velTime);
% plot(time, vlDesLim,time, vlReal);
% legend('soll', 'ist');