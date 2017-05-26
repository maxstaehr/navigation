close all; figure;axis equal;hold on; grid on; 
tx = trax{1};
ty = tray{1};
%plot(P(:,1), P(:,2),'xr');
plot(P2(:,1), P2(:,2),'om');
plot(tx, ty);
% plot(V(1,:));

%  h1 = plot(nan, nan, 'og', 'MarkerSize', 25);
%  h2 = plot(nan, nan, 'xr');
%  for i=2:length(velTime)
%     PA = pfStates{i} ;
%      set(h2, 'XData', PA(1,:));
%      set(h2, 'YData', PA(2,:));
%      set(h1, 'XData', P2(i,1));
%      set(h1, 'YData', P2(i,2));
%      axis equal;
%      pause(0.1);
%      
%   % % plot(P(startIndex:endIndex,1), P(startIndex:endIndex,2), 'xr');
%  
%  end
% figure; hold on; grid on;
% time = timeVelocity - min(timeVelocity);
% plot(time,vlDes, 'DisplayName','vlDes');hold all;plot(time,vlDesLim, 'DisplayName','vlDesLim');plot(time,vlReal, 'DisplayName','vlReal');hold off;
