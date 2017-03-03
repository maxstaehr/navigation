clear all;
load('testData.mat', 'Omega', 'ASensor');
xmin = [0 0]';
xmax = [3 3]';
% [x,fval,exitflag] = ... 
%    fminbnd(@costFunctionLage,xmin,xmax,optimset('TolX',1e-12,'Display','iter'));

c = costFunctionLage( [2 0] );
[x,fval,exitflag] = fminsearch(@costFunctionLage,[5], optimset('Display','iter', 'MaxFunEvals', 1e10, 'TolX', 1e-60, 'TolFun', 1e-60));
disp(x);
% options = saoptimset('Display','iter', 'MaxFunEvals', 1e10, 'TolFun', 1e-60);
% [x,fval,exitflag] = simulannealbnd(@costFunctionLage,[0 0], [0 0], [5 5], options);
% disp(x);
