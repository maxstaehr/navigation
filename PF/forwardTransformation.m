function [ tr ] = forwardTransformation( t1, t2 )
%FORWARDTRANSFORMATION Summary of this function goes here
%   Detailed explanation goes here

e1 = rot2(t1(3))*[t2(1) t2(2)]';
err = bsxfun(@plus, e1 , [t1(1) t1(2)]');
errr = t1(3) + t2(3);
tr = [err' errr];   
end

