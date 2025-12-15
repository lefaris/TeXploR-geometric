% Plots phi values in 1 degree increments in relation to input theta angles
% Shows only simulated values for all 4 cases

tic
k = 1;
l = 1;
the1 = zeros(8000);
the2 = zeros(8000);
phi1c1 = zeros(8000);
phi1c2 = zeros(8000);
phi1c3 = zeros(8000);
phi1c4 = zeros(8000);
for i = 0:2:180
    for j = 0:2:180
        TExploRStatics(i,j)
        the1(k,l) = i;
        the2(k,l) = j;
        phi1c1(k,l) = rad2deg(ans.staticPoses.Case(1).phi1);
        phi1c2(k,l) = rad2deg(ans.staticPoses.Case(2).phi1);
        phi1c3(k,l) = rad2deg(ans.staticPoses.Case(3).phi2);
        phi1c4(k,l) = rad2deg(ans.staticPoses.Case(4).phi2);
        k = k+1;
        l = l+1;
    end
end
% 
% theta1 = [0 0 0 0 0; 45 45 45 45 45; 90 90 90 90 90; 135 135 135 135 135; 180 180 180 180 180];
% theta2 = [0 45 90 135 180; 0 45 90 135 180; 0 45 90 135 180; 0 45 90 135 180; 0 45 90 135 180];
% phi1c1sim = [90 145 135 145 180; 45 90 158 180 45; 90 90 90 90 90; 135 180 23 90 135; 180 35 45 35 90];
% phi1c2sim = [90 145 135 145 180; 45 90 158 180 45; 90 90 90 90 90; 135 180 23 90 135; 180 35 45 35 90];
% phi1c3sim = [90 45 90 135 180; 145 90 90 180 35; 135 158 90 23 45; 145 180 90 90 35; 180 45 90 135 90];
% phi1c4sim = [90 45 90 135 180; 145 90 90 180 35; 135 158 90 23 45; 145 180 90 90 35; 180 45 90 135 90];


theta1 = reshape(diag(the1),91,91);
theta2 = reshape(diag(the2),91,91);
phi1c1sim = reshape(diag(phi1c1),91,91);
phi1c2sim = reshape(diag(phi1c2),91,91);
phi1c3sim = reshape(diag(phi1c3),91,91);
phi1c4sim = reshape(diag(phi1c4),91,91);



figure
subplot(4,1,1)
surf(theta1,theta2,phi1c1sim);
grid on
xlabel('\theta_1')
ylabel('\theta_2')
zlabel('\phi_1')
title('Case 1 where \phi_2 = 0')
hold on

subplot(4,1,2)
surf(theta1,theta2,phi1c2sim);
grid on
xlabel('\theta_1')
ylabel('\theta_2')
zlabel('\phi_1')
title('Case 2 where \phi_2 = 180')
hold on

subplot(4,1,3)
surf(theta1,theta2,phi1c3sim);
grid on
xlabel('\theta_1')
ylabel('\theta_2')
zlabel('\phi_2')
title('Case 3 where \phi_1 = 0')
hold on

subplot(4,1,4)
surf(theta1,theta2,phi1c4sim);
grid on
xlabel('\theta_1')
ylabel('\theta_2')
zlabel('\phi_2')
title('Case 4 where \phi_1 = 180')

toc