function [c,ceq] = fminconstr(x)
phi1 = x(1);
phi2 = x(2);
 c = [wrapTo2Pi(phi1)-pi;-wrapTo2Pi(phi1);wrapTo2Pi(phi2)-pi;-wrapTo2Pi(phi2)];
ceq = staticsTexplorer(x);
end