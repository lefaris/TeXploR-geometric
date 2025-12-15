

% some in-line functions
%so3 = @(x) [0,-x(3),x(2);x(3),0,-x(1);-x(2),x(1),0];
%se3 = @(S) [so3(S(1:3)),S(4:6);zeros(1,4)]
%G = @(theta,omega) (eye(3)*theta + (1-cos(theta))*so(omega)+(theta-sin(theta))*so(omega)^2);
%Rot = @(theta,omegas) eye(3)+ (so3(omegas)*sin(theta))+ (so3(omegas)^2*(1-cos(theta)));
%T = @(omega,v,theta) [Rot(theta,omega),G(theta,omega)*v;zeros(1,3),1];
%adjointT = @(T) [T(1:3,1:3), zeros(3,3); so3(T(1:3,4))*T(1:3,1:3), T(1:3,1:3)]

classdef TExploRDerivationsFunc
methods (Static)
function cls = setup
syms t real positive
syms theta phi M [2 1] real
syms r real
syms mg Mg Fr1 Fr2 real
%% Kinematics


ptonlink = @(r,alpha) r*[cos(alpha),sin(alpha),0,1/r]';
tangentonlink = @(r,alpha) r*[-sin(alpha),cos(alpha),0,0]';
binormalonlink = @(r,alpha) -r*[cos(alpha),sin(alpha),0,0]';
cls.pB1 = ptonlink(r,theta(1));
cls.qB1 = ptonlink(r,phi(1));
cls.tB1 = tangentonlink(r,phi(1));
cls.bB1 = binormalonlink(r,phi(1));
cls.rB1 = 0.15*[0,1,0,1/0.15]';

cls.T12 = [0,0,1,0;
    0,-1,0,0;
    1,0,0,0;
    0,0,0,1];
cls.rB2 = cls.T12*cls.rB1;
cls.pB2 = cls.T12*ptonlink(r,theta(2));
cls.qB2 = cls.T12*ptonlink(r,phi(2));
cls.tB2 = cls.T12*tangentonlink(r,phi(2));
cls.bB2 = cls.T12*binormalonlink(r,phi(2));
end
% *Holonomic constraint*
% Case 1: Both tangent vectors $\mathbf{t}_i$ at points $Q_i$ exist

%simplify((qB1(1:3)-qB2(1:3))'*LieGroup.vec2so3(tB1(1:3))*tB2(1:3))
%% 
% $\phi_1=-\phi_2$ or $\phi_i=0,\pi$

%zB0 = LieGroup.vec2so3(tB1(1:3))*tB2(1:3);
%simplify(zB0'*zB0);
%zB0a = simplify(subs(zB0,phi1,-phi2));
%simplify(zB0a'*zB0a);
%q2Ba = subs(qB2,phi2,-phi1);
%maxmin0 = simplify(zB0a'*bB2(1:3));


% function [phi1Areturn, zB0aReturn, Fr1a1, Fr1a2] = c1a(m,M,g,t1,t2)
% cls = TExploRDerivationsFunc.setup;
% syms t real positive
% syms theta phi [2 1] real
% syms r real
% syms mg Mg Fr1 Fr2 real
% %phi2a = 0;
% zB0a = simplify(subs(LieGroup.vec2so3(cls.tB1(1:3))*cls.tB2(1:3),phi2,-phi1))/(r^2);
% Wrench1a = simplify([2*mg+2*Mg-Fr1-Fr2;(2*mg*LieGroup.vec2so3(cls.rB1(1:3)+cls.rB2(1:3))+2*Mg*LieGroup.vec2so3(cls.pB1(1:3)+cls.pB2(1:3))-...
%     Fr1*LieGroup.vec2so3(cls.qB1(1:3))-Fr2*LieGroup.vec2so3(subs(cls.qB2(1:3),phi2,-phi1)))*zB0a])
% Fcoeffs1a = [coeffs(Wrench1a(1),{Fr1,Fr2});coeffs(Wrench1a(2),{Fr1,Fr2});coeffs(Wrench1a(3),{Fr1,Fr2})];
% big1aA = Fcoeffs1a(:,2:3); smallB1a = Fcoeffs1a(:,1);
% Frs1a = simplify(pinv(big1aA)*smallB1a)
% %Fr12a = solve(Wrench2a(1:2),{Fr1,Fr2});
% %Wrench2aa = simplify(subs(Wrench2a,{Fr1,Fr2},{Fr12a.Fr1,Fr12a.Fr2})/(Mg*r));
% Wrench1aa = simplify(subs(Wrench1a,{Fr1,Fr2},{Frs1a(1),Frs1a(2)})/(Mg*r));
% %simplify(Wrench2aa(3)*sin(phi1)/2);
% %expand(Wrench2aa(3));
% eqn = simplify(Wrench1aa(3));
% p1a = solve(eqn,phi1,'Real',true)
% %if (cos(t1) < 6.1232e-16 && cos(t1) > -6.1232e-16)
% %if ((cos(t1) < 10e-16) && (cos(t1) > -6.1232e-17))
% if (t1 > 1.5 && t1 < 1.6)
%     phi1Areturntemp = 0;
% else
%     phi1Areturntemp = simplify(subs(p1a(1),{theta1,theta2},{t1,t2}));
% end
% phi1Areturn = (1 - sign(phi1Areturntemp)) * pi / 2 + phi1Areturntemp
% zB0aReturn = subs(zB0a,phi1,phi1Areturn)
% Frs1a = subs(Frs1a,{Mg,mg,phi1,theta1,theta2},{M*-g,m*-g,phi1Areturn,t1,t2})
% Fr1a1 = Frs1a(1);
% Fr1a2 = Frs1a(2);
% end


% Case 2: $\mathbf{t}_1$exists but $\mathbf{t}_2$ doesn't exist

function [phi2Areturn, zB1aReturn, Fr2a1, Fr2a2] = c2a(m,M,g,t1,t2)
cls = TExploRDerivationsFunc.setup;
syms t real positive
%syms theta phi [2 1] real
syms theta phi [2 1] real
%theta1 = double(subs(stheta1))
%theta2 = double(subs(stheta2))
syms r real
syms mg Mg Fr1 Fr2 real
phi2a = 0;
zB1a = simplify(LieGroup.vec2so3(cls.tB1(1:3))*subs((cls.qB1(1:3)-cls.qB2(1:3)),phi2,phi2a))/(sqrt(2)*r^2)
Wrench2a = simplify([2*mg+2*Mg-Fr1-Fr2;(2*mg*LieGroup.vec2so3(cls.rB1(1:3)+cls.rB2(1:3))+2*Mg*LieGroup.vec2so3(cls.pB1(1:3)+cls.pB2(1:3))-...
    Fr1*LieGroup.vec2so3(cls.qB1(1:3))-Fr2*LieGroup.vec2so3(subs(cls.qB2(1:3),phi2,phi2a)))*zB1a])
Fcoeffs2a = [coeffs(Wrench2a(1),{Fr1,Fr2});coeffs(Wrench2a(2),{Fr1,Fr2});coeffs(Wrench2a(3),{Fr1,Fr2})];
big2aA = Fcoeffs2a(:,2:3); smallB2a = Fcoeffs2a(:,1);
Frs2a = simplify(pinv(big2aA)*smallB2a)
%Fr12a = solve(Wrench2a(1:2),{Fr1,Fr2});
%Wrench2aa = simplify(subs(Wrench2a,{Fr1,Fr2},{Fr12a.Fr1,Fr12a.Fr2})/(Mg*r));
Wrench2aa = simplify(subs(Wrench2a,{Fr1,Fr2},{Frs2a(1),Frs2a(2)})/(Mg*r));
%simplify(Wrench2aa(3)*sin(phi1)/2);
%expand(Wrench2aa(3));
eqn = simplify(Wrench2aa(3));
p2a = solve(eqn,phi1,'Real',true)

%p2a(1) = double(subs(p2a(1)))
%if (cos(t1) < 6.1232e-16 && cos(t1) > -6.1232e-16)
%if ((cos(t1) < 10e-16) && (cos(t1) > -6.1232e-17))
%theta2 = double(subs(theta2))
if (t1 > 1.5 && t1 < 1.6)
    phi2Areturntemp = 0;
else
    phi2Areturntemp = atan((sin(t1) - sin(t2))/cos(t1))
    %phi2Areturntemp = double(subs(p2a(1),{theta1,theta2},{t1,t2}))
end
phi2Areturn = (1 - sign(phi2Areturntemp)) * pi / 2 + phi2Areturntemp
zB1aReturn = subs(zB1a,phi1,phi2Areturn)
Frs2a = subs(Frs2a,{Mg,mg,phi1,theta1,theta2},{M*-g,m*-g,phi2Areturn,t1,t2})
Fr2a1 = Frs2a(1);
Fr2a2 = Frs2a(2);
end

%% 
% *Solution for Case 2a: $\phi_2=0$ and $\tan(\phi_1) = \frac{\sin(\theta_1)-\sin(\theta_2)}{\cos(\theta_1)}$

function [phi2Breturn, zB1bReturn, Fr2b1, Fr2b2] = c2b(m,M,g,t1,t2)
cls = TExploRDerivationsFunc.setup;
syms t real positive
syms theta phi [2 1] real
syms r real
syms mg Mg Fr1 Fr2 real
phi2b = pi;
zB1b = simplify(LieGroup.vec2so3(cls.tB1(1:3))*subs((cls.qB1(1:3)-cls.qB2(1:3)),phi2,phi2b)/(sqrt(2)*-r^2));
Wrench2b = simplify([2*mg+2*Mg-Fr1-Fr2;(2*mg*LieGroup.vec2so3(cls.rB1(1:3)+cls.rB2(1:3))+2*Mg*LieGroup.vec2so3(cls.pB1(1:3)+cls.pB2(1:3))-...
    Fr1*LieGroup.vec2so3(cls.qB1(1:3))-Fr2*LieGroup.vec2so3(subs(cls.qB2(1:3),phi2,phi2b)))*zB1b])
Fcoeffs2b = [coeffs(Wrench2b(1),{Fr1,Fr2});coeffs(Wrench2b(2),{Fr1,Fr2});coeffs(Wrench2b(3),{Fr1,Fr2})];
big2bA = Fcoeffs2b(:,2:3); smallB2b = Fcoeffs2b(:,1);
Frs2b = simplify(pinv(big2bA)*smallB2b)
%Fr12b = solve(Wrench2b(1:2),{Fr1,Fr2});
Wrench2bb = simplify(subs(Wrench2b,{Fr1,Fr2},{Frs2b(1),Frs2b(2)})/(Mg*r));
eqn = simplify(Wrench2bb(3));
p2b = solve(eqn,phi1,'Real',true)
%if (cos(t1) < 6.1232e-16 && cos(t1) > -6.1232e-16)
%if ((cos(t1) < 10e-16) && (cos(t1) > -6.1232e-17))
if (t1 > 1.5 && t1 < 1.6)
    phi2Breturntemp = 0;
else
    phi2Breturntemp = atan((sin(t1) - sin(t2))/cos(t1))
end
phi2Breturn = (1 - sign(phi2Breturntemp)) * pi / 2 + phi2Breturntemp;
zB1bReturn = subs(zB1b,phi1,phi2Breturn)
Frs2b = subs(Frs2b,{Mg,mg,phi1,theta1,theta2},{M*-g,m*-g,phi2Breturn,t1,t2})
Fr2b1 = Frs2b(1);
Fr2b2 = Frs2b(2);
end
%expand(Wrench2aa(3));
%test = subs(Wrench2b,{theta(1),theta(2)},{3*pi/4,0});
%simplify(test(2)*sqrt(2)/r);
%aa = solve(test([1,2]),{Fr1,Fr2});
%aa.Fr1;
%aa.Fr2;
%% 
% *Solution for Case 2b:* $\tan(\phi_1)=\frac{\sin(\theta_1)-\sin(\theta_2)}{\cos(\theta_1)}$
% 
% This is along the vector from the $Q2$ where $\phi_2=0$ or $\pi$
% Case 3: $\mathbf{t}_2$ exists but $\mathbf{t}_1$ doesn't exist

function [phi3Areturn, zB2aReturn, Fr3a1, Fr3a2] = c3a(m,M,g,t1,t2)
cls = TExploRDerivationsFunc.setup;
syms t real positive
syms theta phi [2 1] real
syms r real
syms mg Mg Fr1 Fr2 real
phi1a=0;
zB2a = simplify(LieGroup.vec2so3(cls.tB2(1:3))*subs((cls.qB1(1:3)-cls.qB2(1:3)),phi1,phi1a)/(-r^2));
Wrench3a = simplify([2*mg+2*Mg-Fr1-Fr2;(2*mg*LieGroup.vec2so3(cls.rB1(1:3)+cls.rB2(1:3))+2*Mg*LieGroup.vec2so3(cls.pB1(1:3)+cls.pB2(1:3))-...
    Fr1*LieGroup.vec2so3(subs(cls.qB1(1:3),phi1,phi1a))-Fr2*LieGroup.vec2so3(cls.qB2(1:3)))*zB2a])
Fcoeffs3a = [coeffs(Wrench3a(1),{Fr1,Fr2});coeffs(Wrench3a(3),{Fr1,Fr2});coeffs(Wrench3a(4),{Fr1,Fr2})];
big3aA = Fcoeffs3a(:,2:3); smallB3a = Fcoeffs3a(:,1);
Frs3a = simplify(pinv(big3aA)*smallB3a)
Wrench3aa = simplify(subs(Wrench3a,{Fr1,Fr2},{Frs3a(1),Frs3a(2)})/(2*Mg*r^3))
eqn = simplify(Wrench3aa(3));
p3a = solve(eqn,phi2,'Real',true)
%if (cos(t2) < 6.1232e-16 && cos(t2) > -6.1232e-16)
%if ((cos(t2) < 10e-16) && (cos(t2) > -6.1232e-17))
if (t2 > 1.5 && t2 < 1.6)
    phi3Areturntemp = 0;
else
    phi3Areturntemp = -atan((sin(t1) - sin(t2))/cos(t2))
end
phi3Areturn = (1 - sign(phi3Areturntemp)) * pi / 2 + phi3Areturntemp
zB2aReturn = subs(zB2a,phi2,phi3Areturn)
Frs3a = subs(Frs3a,{Mg,mg,phi2,theta1,theta2},{M*-g,m*-g,phi3Areturn,t1,t2})
Fr3a1 = Frs3a(1);
Fr3a2 = Frs3a(2);
end
%% 
% *Solution 3a:* $\tan{\phi_2} = \frac{\sin(\theta_2)-\sin(\theta_1)}{\cos(\theta_1)}$

function [phi3Breturn, zB2bReturn, Fr3b1, Fr3b2] = c3b(m,M,g,t1,t2)
cls = TExploRDerivationsFunc.setup;
syms t real positive
syms theta phi [2 1] real
syms r real
syms mg Mg Fr1 Fr2 real
phi1b = pi;
%zB2b = simplify(LieGroup.vec2so3(cls.tB2(1:3))*subs((cls.qB1(1:3)-cls.qB2(1:3)),phi1,phi1b)/(sqrt(2)*r^2));
zB2b = simplify(LieGroup.vec2so3(cls.tB2(1:3))*subs((cls.qB1(1:3)-cls.qB2(1:3)),phi1,phi1b)/(r^2));
%zB2b = simplify(subs((qB1(1:3)-qB2(1:3))'*LieGroup.vec2so3(tB2(1:3)),phi1,pi))
Wrench3b = simplify([2*mg+2*Mg-Fr1-Fr2;(2*mg*LieGroup.vec2so3(cls.rB1(1:3)+cls.rB2(1:3))+2*Mg*LieGroup.vec2so3(cls.pB1(1:3)+cls.pB2(1:3))-...
    Fr1*LieGroup.vec2so3(subs(cls.qB1(1:3),phi1,phi1b))-Fr2*LieGroup.vec2so3(cls.qB2(1:3)))*zB2b])
Fcoeffs3b = [coeffs(Wrench3b(1),{Fr1,Fr2});coeffs(Wrench3b(3),{Fr1,Fr2});coeffs(Wrench3b(4),{Fr1,Fr2})];
big3bA = Fcoeffs3b(:,2:3); smallB3b = Fcoeffs3b(:,1);
Frs3b = simplify(pinv(big3bA)*smallB3b)
%Wrench3bb = simplify(subs(Wrench3b,{Fr1,Fr2},{Frs3b(1),Frs3b(2)})/(2*Mg*r^3))
Wrench3bb = simplify(subs(Wrench3b,{Fr1,Fr2},{Frs3b(1),Frs3b(2)})/(Mg*r))
eqn = simplify(Wrench3bb(3));
p3b = solve(eqn,phi2,'Real',true)
%if (cos(t2) < 6.1232e-16 && cos(t2) > -6.1232e-16)
%if ((cos(t2) < 10e-16) && (cos(t2) > -6.1232e-17))
if (t2 == (pi/2))
    phi3Breturntemp = 0
else
    %digits(32);
    phi3Breturntemp = -atan((sin(t1) - sin(t2))/cos(t2))
end
%phi3Breturntemp = subs(p3b(2),{theta1,theta2},{t1,t2})
phi3Breturn = (1 - sign(phi3Breturntemp)) * pi / 2 + phi3Breturntemp
zB2bReturn = subs(zB2b,phi2,phi3Breturn)
Frs3b = subs(Frs3b,{Mg,mg,phi2,theta1,theta2},{M*-g,m*-g,phi3Breturn,t1,t2})
Fr3b1 = Frs3b(1);
Fr3b2 = Frs3b(2);
end

end
end
%% 
% This is along the vector from the $Q1$ where $\phi_2=0$ or $\pi$
% 
% 

% syms Rsb [3 3] real
% syms omega [3 1] real
% syms theta real
% Rsb2 = LieGroup.MatExponential3(omega,theta);
% zB = Rsb'*[0,0,1]';
% zB2 = Rsb2'*[0,0,1]';
%% 
% E1: Tangent at $Q_1$ is normal to $\mathbf{z}$
% 
% Tangent at point $Q_i$ is $\mathbf{t}_i(\phi_i)=\left.\frac{\partial q_i(\alpha)}{\partial 
% \alpha}\right|_{\alpha = \phi_i}$

% E1 = simplify(tB1(1:3)'*zB);
% E2 = zB'*tB2(1:3);
% E3 = zB'*(qB1(1:3)-qB2(1:3));
% aa = solve(E1,Rsb(3,1));
% bb = solve(E2,Rsb(3,3));
% simplify(subs(E3,{Rsb(3,1),Rsb(3,3)},{aa,bb}));
% vv = simplify([aa;Rsb(3,2);bb]);
% simplify(vv'*vv);
% cc = solve(E1,Rsb(3,2));
% dd=solve(subs(E2,Rsb(3,2),cc),Rsb(3,3));
% ww = simplify([Rsb(3,1);cc;dd]);
% simplify(ww'*ww);
% ee = solve(E2,Rsb(3,2));
% ff = solve(subs(E1,Rsb(3,2),ee),Rsb(3,1));
% xx = [ff;ee;Rsb(3,3)];
% xx'*xx;
% % Non-holonomic constraint
% % This will constraint the solution, i.e., twist
% 
% syms omega_b v_b [3 1] real
% Vb = [omega_b;v_b];
% %% 
% zero velocity at points $Q_1,Q_2$ where $\mathbf{\dot{q}}_i^b = [V_b]\mathbf{q}_i^b$ 
% (homogenous form)
% 
% $\mathbf{\dot{q}}_i^b = \mathbf{v}_b + [\mathbf{\omega}_b]\mathbf{q}_i^b$. 
% Dropping the superscript of $b$ and solving for $\mathbf{v}_b,\mathbf{\omega}_b$
% 
% (1)-(2): $\left(\mathbf{q}_1-\mathbf{q}_2\right)\times \mathbf{\omega}_b=0 
% \Rightarrow \mathbf{\omega}_b=\alpha\left(\mathbf{q}_1-\mathbf{q}_2\right)$
% 
% (1): $\mathbf{v}_b=\mathbf{q}_1\times \mathbf{\omega}_b=\mathbf{q}_2\times 
% \mathbf{\omega}_b=\alpha \mathbf{q}_2\times \mathbf{q}_1$
% 
% Hence: $V_b=\alpha\left[\left(\mathbf{q}_1-\mathbf{q}_2\right);\mathbf{q}_2\times 
% \mathbf{q}_1\right]$
% 
% *Note:* This solution to $V_b$ implies constraint on solution space, i.e., 
% $V_b$ cannot just take any value, rather a scalar multiple of the stated.
% 
% *Comment:*
% 
% The same holds true in the inertial coordiante system $\{s\}$
% 
% $\mathbf{\dot{q}}_i^s = \mathbf{v}_s + [\mathbf{\omega}_s]\mathbf{q}_i^s$. 
% 
% (1)-(2): $\left(\mathbf{q}_1^s-\mathbf{q}_2^s\right)\times \mathbf{\omega}_s=0 
% \Rightarrow \mathbf{\omega}_s=\alpha\left(\mathbf{q}_1^s-\mathbf{q}_2^s\right)$
% 
% (1): $\mathbf{v}_s=\mathbf{q}_1^s\times \mathbf{\omega}_s=\mathbf{q}_2^s\times 
% \mathbf{\omega}_s=\alpha \mathbf{q}_2^s\times \mathbf{q}_1^s$
% 
% Hence: $V_s=\alpha\left[\left(\mathbf{q}_1^s-\mathbf{q}_2^s\right);\mathbf{q}_2^s\times 
% \mathbf{q}_1^s\right]$

%  From above
% syms alpha real
% omegaB = alpha*(qB1(1:3)-qB2(1:3));
% simplify(LieGroup.vec2so3(qB1(1:3))*omegaB);
% simplify(LieGroup.vec2so3(qB2(1:3))*omegaB);
% simplify(LieGroup.vec2so3(qB1(1:3))*omegaB - LieGroup.vec2so3(qB2(1:3))*omegaB);
% Vb = alpha*[qB1(1:3)-qB2(1:3);so3(qB2(1:3))*qB1(1:3)]
% VbBraket = LieGroup.vec2se3(Vb)
% vGeometric = VbBraket*[0,0,0,1]';
% % The solution of Vb should imply zero velocity of the points Q1,Q2
% qB1_dot = simplify(VbBraket*qB1);
% qB2_dot = simplify(VbBraket*qB2);
% Geometric interpretation
% The instantaneous center of rotation and pitch of the screw motion are 
% 
% $h=\frac{\mathrm{\omega}_b^T\mathbf{v}_b}{|\mathrm{\omega}_b|^2}$ and $\rho 
% = \frac{\mathrm{\omega}_b\times \mathbf{v}_b}{|\mathrm{\omega}_b|^2}$
% 
% where $\mathbf{\omega}_b=\alpha\left(\mathbf{q}_1-\mathbf{q}_2\right), \mathbf{v}_b=\alpha 
% \mathbf{q}_2\times \mathbf{q}_1$
% 
% $\Rightarrow h=0$, i.e., no pitch of the screw
% 
% $$\Rightarrow \rho = \frac{\left([\mathbf{q}_1]-[\mathbf{q}_2]\right)[\mathbf{q}_2]\mathbf{q}_1}{|(\mathbf{q}_1-\mathbf{q}_2)|^2} 
% = -\left(\frac{[\mathbf{q}_1]^2\mathbf{q}_2 + [\mathbf{q}_2]^2\mathbf{q}_1}{|\mathbf{q}_1-\mathbf{q}_2|^2}\right)$$

% h = simplify(Vb(1:3)'*Vb(4:6)); % zero pitch means pure rotation about the point
% rho = simplify(LieGroup.vec2so3(Vb(1:3))*Vb(4:6)/(Vb(1:3)'*Vb(1:3)));
% %mid-point between q1, q2 is q1 + (q2-q1)/2 or (q1+q2)/2 
% qB1 + (qB2-qB1)/2;
% syms phi1Dot phi2Dot alphaDot real
% VbDot = diff(Vb,phi(1))*phi1Dot + diff(Vb,phi(2))*phi2Dot + diff(Vb,alpha)*alphaDot;
% simplify([diff(Vb,phi(1))';diff(Vb,phi(2))';diff(Vb,alpha)']);
% VbDotBracket = LieGroup.vec2se3(VbDot);
% aB1 = simplify((VbDotBracket + VbBraket^2)*qB1);
% aB2 = simplify((VbDotBracket + VbBraket^2)*qB2);
% aB12=simplify((aB1-aB2)/(alpha*r^2));
% phi1DotStar = solve(aB12(2),phi1Dot);
% % phi2DotStar=0 OR
% simplify([subs(aB12(1),phi1Dot,phi1DotStar);subs(aB12(3),phi1Dot,phi1DotStar)]/phi2Dot);
% aB1;
% aa = simplify(LieGroup.vec2so3(Vb(1:3))^2 + LieGroup.vec2so3(VbDot(1:3)));
% bb = simplify(aa*(qB1(1:3)-qB2(1:3))/(-alpha*r^2));
% simplify(solve(bb(2),phi1Dot));
% % Solution not possible
% cc = simplify([subs(bb(1),phi1Dot,solve(bb(2),phi1Dot));subs(bb(3),phi1Dot,solve(bb(2),phi1Dot))]/phi2Dot);
% dd1 = simplify(VbDot(4:6) + aa*qB1(1:3));
% subs(dd1,phi(1),pi/2);
% Holonomic constraint
% The points $Q_1,Q_2$ are in contact with the ground
% 
% $\mathbf{q}_i^s = T_{sb}\mathbf{q}_i^b$ and the z-th component is zero
% 
% $$[0,0,1,0]\cdot\mathbf{q}_i^s=0$$
% 
% $T_{sb}=\exp([V_s]t)$ where $V_s=\alpha\left[\left(\mathbf{q}_1^s-\mathbf{q}_2^s\right);\mathbf{q}_2^s\times 
% \mathbf{q}_1^s\right]$
% 
% $R_{sb} = \exp([\omega_s]t) = I + \sin(\theta)[\hat{\omega}_s] + (1-\cos(\theta))[\hat{\omega}_s]^2]$ 
% where $\theta = |\omega_s|t, \hat{\omega}_s=\frac{\omega_s}{|\omega_s|}$
% 
% *Approach 1:*
% 
% $\mathbf{\omega}_s=\alpha\left(\mathbf{q}_1-\mathbf{q}_2\right)$, $\mathbf{v}_s=\alpha 
% \mathbf{q}_2\times \mathbf{q}_1$
% 
% $$R_{sb} = \exp([\mathbf{\omega}_s]t)=\exp(\alpha[\mathbf{q}_1-\mathbf{q}_2]t)$$
% 
% $$\mathbf{p}_{sb}= \left( (I-R_{sb})[\hat{\omega}] + \theta\hat{\omega}\hat{\omega}^T 
% \right)\hat{\mathbf{v}}_s$$

% syms q1s q2s w [2 1] real
% syms v3 real
% qS1 = [q1s;0]; qS2 = [q2s;0];
% omegaS = alpha*(qS1-qS2);
% vS = alpha*LieGroup.vec2so3(qS2)*qS1;
% w = [w1,w2,0]';
% wnorm= sqrt(w'*w);
% v = [0,0,v3]';
% syms beta real
% LieGroup.vec2so3(w);
% Rsb = (Rot(beta,w/wnorm))
% Rsb2=subs(Rsb,wnorm,1)
% simplify(Rsb'*Rsb)
% psb= simplify(((eye(3)-Rsb)*(so3(w)/wnorm) + (beta*w*w'/(w'*w)))*v/wnorm)
% psb2=subs(psb,wnorm,1)
% Tsb = [Rsb,psb;zeros(1,3),1]
% Tsb2 = subs(Tsb,wnorm,1)
%%
% % % % C1 = [0,0,1,0]*Tsb2*qB1
% % % % C2 = [0,0,1,0]*Tsb2*qB2
% % % % subs(C2,wnorm^2,1)
% % % % simplify(C1)
% % % % simplify(subs(C2,wnorm^2,1))
%% Statics and Dynamics 1.0
% Statics
% $f = (m_1+m_2+M_1+M_2-F_1-F_2)g\mathbf{z}$ (1 equation) where $F_i$ are normalized 
% for gravity $g$
% 
% $m = (m_1[\mathbf{r}_{COM,1}] + m_1[\mathbf{r}_{COM,1}]+M_1[\mathbf{p}_1]+M_2[\mathbf{p}_2]-F_1[\mathbf{q}_1]-F_2[\mathbf{q}_2])\mathbf{z}$ 
% (3 equations)
% 
% Holonomic constraint
% 
% $\mathbf{z}^T\mathbf{q}_1=\mathbf{z}^T\mathbf{q}_2=0$ (2 equations)
% 
% Physical constraint
% 
% $$\phi_i\in[-90\deg,90\deg] \forall i=1,2$$
% 
% If we assume $O_b,O_s$ to be conicident then that number of unknowns are 2 
% (twist) + 2 (forces) + 2 (angles of contact point $\phi_i$) =7
% 
% Number of equations = 6
% 
% $$E1 = (m_1+m_2+M_1+M_2-F_1-F_2) = 0$$
% 
% E2-5 : $R_{sb}^T[\mathbf{z}]R_{sb}\left(\mathbf{y} -F_1\mathbf{q}_1-F_2\mathbf{q}_2 
% \right)=0$

% syms m M [2 1]
% rCOM1B = [0,r/3,0,1]';
% rCOM2B = T12*[0,r/3,0,1]';
% LieGroup.MatExponential3(omega,sqrt(omega'*omega))
%% Hybrid state system (Finite State Machines)
% The link edges are identified by $\phi_i=\pm90\deg$. Four states exist
%% 
% # *State 1,2:* $\phi_1=\pm 90\deg$ and $\phi_2(t)$ varies with time.
% # *State 3,4:* $\phi_2=\pm 90\deg$ and $\phi_1(t)$ varies with time.
%% 
% Assuming State 1: $\phi_1=90\deg$ and point of contact $Q_2$ varies with time. 
% 
% qB2s1 = subs(qB2,phi(2),pi/2);
% omegaBs1 = alpha*(qB2s1-qB2);
% simplify(omegaBs1'*omegaBs1);