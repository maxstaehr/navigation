a = [1 1 pi/4]';
b = [1 1 pi/6]';

A = [...
        -1 -1 0;
        1 -1 0;
        1 1 0;
        -1 1 0;
        -1 -1 0;]';

A1 = bsxfun(@plus, rot2(a(3)) * A(1:2,:), a(1:2));
A1 = [A1; A(3,:)+a(3)];
Ainrobot = egoKompensatePunkte(b', [0 0 0],  A1');

close all; figure; hold on; grid on;
line(A1(1,:), A1(2,:), 'Color', 'k');

A1 = bsxfun(@plus, rot2(b(3)) * Ainrobot(:,1:2)', b(1:2));
line(A1(1,:), A1(2,:), 'Color', 'r');

