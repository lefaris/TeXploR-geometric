% Plots phi values in relation to input theta angles

theta1 = [0 0 0 0 0 45 45 45 45 45 90 90 90 90 90 135 135 135 135 135 180 180 180 180 180];
theta2 = [0 45 90 135 180 0 45 90 135 180 0 45 90 135 180 0 45 90 135 180 0 45 90 135 180];
phi1c1 = [0 145 135 145 180 60 70 160 180 70 90 90 90 90 90 135 180 25 115 135 180 35 45 35 165];
phi1c1sim = [90 145 135 145 180 45 90 158 180 45 90 90 90 90 90 135 180 23 90 135 180 35 45 35 90];
subplot(2,2,1)
plot3(theta1,theta2,phi1c1, '-o');
hold on
plot3(theta1,theta2,phi1c1sim, '-x');
legend('Experimental','Simulated');
xlabel('\theta_1')
ylabel('\theta_2')
zlabel('\phi_1')
title('Case 1 where \phi_2 = 0')
%view
hold on

subplot(2,2,2)
phi1c2 = [20 145 140 155 180 60 70 160 180 50 90 90 90 90 90 135 180 25 110 135 180 35 45 35 165];
phi1c2sim = [90 145 135 145 180 45 90 158 180 45 90 90 90 90 90 135 180 23 90 135 180 35 45 35 90];
plot3(theta1,theta2,phi1c2, '-o');
hold on
plot3(theta1,theta2,phi1c2sim, '-x');
legend('Experimental','Simulated');
xlabel('\theta_1')
ylabel('\theta_2')
zlabel('\phi_1')
title('Case 2 where \phi_2 = 180')

hold on

subplot(2,2,3)
phi1c3 = [0 45 90 130 180 145 75 90 180 35 140 160 90 25 45 150 180 90 110 35 180 45 90 135 160];
phi1c3sim = [90 45 90 135 180 145 90 90 180 35 135 158 90 23 45 145 180 90 90 35 180 45 90 135 90];
plot3(theta1,theta2,phi1c3, '-o');
hold on
plot3(theta1,theta2,phi1c3sim, '-x');
legend('Experimental','Simulated');
xlabel('\theta_1')
ylabel('\theta_2')
zlabel('\phi_2')
title('Case 3 where \phi_1 = 0')

hold on

subplot(2,2,4)
phi1c4 = [20 55 90 130 155 145 75 90 180 30 145 160 90 25 45 150 180 90 110 35 180 55 90 135 165];
phi1c4sim = [90 45 90 135 180 145 90 90 180 35 135 158 90 23 45 145 180 90 90 35 180 45 90 135 90];
plot3(theta1,theta2,phi1c4, '-o');
hold on
plot3(theta1,theta2,phi1c4sim, '-x');
legend('Experimental','Simulated');
xlabel('\theta_1')
ylabel('\theta_2')
zlabel('\phi_2')
title('Case 4 where \phi_1 = 180')
