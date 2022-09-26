% main function
clear all
close all
Name = 'Brachistochrone Problem';
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
global nc_defect ndiffeq nnodes nlp_state ncv nlpv x_0 xf2 xf1 x01 x02
% number of differential equations 
ndiffeq = 2; 
% number of control variables 
ncv = 1;
% number of discretization nodes 
nnodes = 20; 
% number of state nlp variables 
nlp_state = ndiffeq * nnodes; 
% number of control nlp variables 
nlp_control = ncv * nnodes; 
% total number of nlp variables 
nlpv = nlp_state + nlp_control; 
% number of state vector defect equality constraints 
nc_defect = nlp_state - ndiffeq; 
% number of auxilary equality constraints (boundary conditions) 
nc_aux = 4; 
% total number of equality constraints 
nc_total = nc_defect + nc_aux;

% Set the starting and ending points
x01 = -1.8;  % starting point
x02 = 0;
xf1 = 10;  % ending point
xf2 = -10;
% set the initial guess/input of NLP variables
u_guess=atan((xf2-x02)/(xf1-x01));
for i=1:1:nnodes
    x_0(2*i-1)=x01+(xf1-x01)/(nnodes-1)*(i-1);
    x_0(2*i)=x02+(xf2-x02)/(nnodes-1)*(i-1);
    x_0(nlp_state+i)=u_guess;
end
x_0(nlpv+1)=3*sqrt(2*9.8*x02);

[X_OUT,FVAL,EXITFLAG,OUTPUT] = fmincon('trapm3_f',x_0,[],[],[],[],[],[],'trapm3_c');
% For the inputs of the fmincon function, 'trapm3_f' is the objective
% function to be minimize and 'trapm3_c' include all constraints
for i=1:1:nnodes
    yff(i)=X_OUT(2*i);
    xff(i)=X_OUT(2*i-1);
    uff(i)=X_OUT(nlp_state+i);
end
figure (1)
plot(xff,yff)
