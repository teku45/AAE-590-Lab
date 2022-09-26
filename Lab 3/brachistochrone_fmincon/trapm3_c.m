function [c, ceq]= trapm3_c (x) 
% equality constraints 
% trapezoidal collocation method 
% inputs 
% x = current nlp variable values 
% outputs 
% c = vector of equality constraints evaluated at x 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
global nc_defect ndiffeq nnodes nlp_state ncv nlpv tarray x_0 xf2 xf1 x01 x02
% compute state vector defect equality constraints 
tarray(1) = 0; 
tarray(nnodes) = x(nlpv+1); 
deltat = x(nlpv+1) / (nnodes - 1); 
for i = 2:1:nnodes - 1 
    tarray(i) = (i - 1) * deltat;
end
for k = 1:1:nnodes - 1 
    % time elements 
    tk = tarray(k); 
    tkp1 = tarray(k + 1); 
    % state vector elements 
    if k == 1
        % first node 
        nks = 0; 
        nkp1s = ndiffeq; 
    else
        % reset to previous node 
        nks = (k - 1) * ndiffeq; 
        nkp1s = nks + ndiffeq; 
    end
    for i = 1:1:ndiffeq 
        xk(i) = x(nks + i); 
        xkp1(i) = x(nkp1s + i); 
    end
    % control variable elements
    if (k == 1) 
        % first node 
        nkc = nlp_state; 
        nkp1c = nkc + ncv; 
    else
        % reset to previous node 
        nkc = nlp_state + (k - 1) * ncv; 
        nkp1c = nkc + ncv; 
    end
    for i = 1:1:ncv 
        uk(i) = x(nkc + i); 
        ukp1(i) = x(nkp1c + i); 
    end
    % compute state vector defects for current node 
    reswrk = trap_defect(tk, tkp1, xk, xkp1, uk, ukp1); 
    % load defect array for this node 
    for i = 1:1:ndiffeq 
        resid(nks + i) = reswrk(i); 
    end
end
% set active defect constraints 
% (offset by 1) 
for i = 1:1:nc_defect 
    ceq(i) = resid(i); 
end
% current final state vector 
xfinal(1) = x(nlp_state - 1); 
xfinal(2) = x(nlp_state); 

% current initial state vector
xinitial(1) = x(1); 
xinitial(2) = x(2); 

% initial boundary conditions
ceq(nc_defect + 1) = xinitial(1)-x01; 
ceq(nc_defect + 2) = xinitial(2)-x02; 
% final boundary conditions
ceq(nc_defect + 3) = xfinal(1)-xf1; 
ceq(nc_defect + 4) = xfinal(2)-xf2; 
c=[];
