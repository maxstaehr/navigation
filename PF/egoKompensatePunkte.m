function [ Xout ] = egoKompensatePunkte(x_neu, x_alt,  Xin)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

% x_res = x_neu - x_alt;
% %%transforming into world
% X_temp = rot2(x_alt(3))*Xin(:,1:2)';
% X_temp = X_temp';
% T1 = bsxfun(@plus,X_temp,x_alt(1:2));
% %transforming from world into neu
% 
% T2 = rot2(-x_neu(3))*T1';
% T2 = T2';
% Xout = bsxfun(@minus,T2,x_neu(1:2));
% Xout = [Xout Xin(:,3)-x_res(3)];

x_res = x_neu - x_alt;
x_temp = rot2(-x_alt(3))*[x_res(1:2)]';
T1 = bsxfun(@minus,Xin(:,1:2),x_temp');
Xout = rot2(-x_res(3))*T1';
Xout = Xout';
Xout = [Xout Xin(:,3)-x_res(3)];
end

