function f = trapm3_f (x) 
% objective function 
% trapezoidal collocation method 
% inputs 
% x = current nlp variable values 
% outputs 
% f = objective function evaluated at x 
global nlpv

tfinal = x(nlpv+1); 
% objective function (maximize final radius)
f = tfinal;