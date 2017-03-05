a = [1 1 pi/4]';
b = [1 1 pi/6]';

A = [0 0;
       1 0]';

A1 = bsxfun(@plus, rot2(a(3)) * A, a(1:2));

close all; figure; hold on; grid on;
line(A(1,:), A(2,:), 'Color', 'k');
line(A1(1,:), A1(2,:), 'Color', 'r');

