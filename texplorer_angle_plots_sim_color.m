% Agile Robotics Laboratory at UA
% TExploR Project
% Date: 04/24/2024
% 
% Plots phi values in 2 degree increments in relation to input theta angles
% from 0 - pi. Shows only simulated values for all 4 cases
% 
% Usage:
% Click run!
% If you want to modify the resolution - how many degrees you are 
% plotting - change the for loop increment as well as lines 41-46
% where it reshapes the phi1 and phi2 vectors into diagonal arrays

tic
k = 1;
l = 1;

% Initialize theta and phi vectors to save memory
% the1 = zeros(8000);
% the2 = zeros(8000);
% phi1c1 = zeros(8000);
% phi1c2 = zeros(8000);
% phi1c3 = zeros(8000);
% phi1c4 = zeros(8000);

for i = 0:1:180 % For theta1 values - changing from 0:2:180
    for j = 0:1:180 % For theta2 values - changing from 0:2:180
        TExploRStaticsFastPlotting(i,j) % Run TExploreR sim code for each theta combo
        the1(k) = i; % Theta1
        the2(k) = j; % Theta2
        phi1c1(k) = rad2deg(ans.staticPoses.Case(1).phi1); % Phi1 case 1
        phi1c2(k) = rad2deg(ans.staticPoses.Case(2).phi1); % Phi1 case 2
        phi1c3(k) = rad2deg(ans.staticPoses.Case(3).phi2); % Phi2 case 3
        phi1c4(k) = rad2deg(ans.staticPoses.Case(4).phi2); % Phi2 case 4
        k = k+1;
        l = l+1;
    end
end

% Surf plot phi1 for cases 1 & 2 and phi2 for cases 3 & 4
%figure

tiledlayout(2,2);
nexttile
% ax1 = subplot(4,1,1)
% gscatter(ax1,the1,the2,phi1c1)
scatter(the1,the2,10,phi1c1, 'filled')
xlim([0 180])
ylim([0 180])
xticks([0, 45, 90, 135, 180])
yticks([0, 45, 90, 135, 180])
axis square

% colorbar;
% surf(theta1,theta2,phi1c1sim);
grid on
grid minor
xlabel('\theta_1')
ylabel('\theta_2')
% zlabel('\phi_1')
title('State 1: \phi_2 = 0')


hold on
nexttile
scatter(the1,the2,10,phi1c2,'filled')
xlim([0 180])
ylim([0 180])
xticks([0, 45, 90, 135, 180])
yticks([0, 45, 90, 135, 180])
axis square
grid on
grid minor
xlabel('\theta_1')
ylabel('\theta_2')
title('State 2: \phi_2 = 180')
hold on

nexttile
scatter(the1,the2,10,phi1c3,'filled')
xlim([0 180])
ylim([0 180])
xticks([0, 45, 90, 135, 180])
yticks([0, 45, 90, 135, 180])
axis square
grid on
grid minor
xlabel('\theta_1')
ylabel('\theta_2')
title('State 3: \phi_1 = 0')
hold on

nexttile
scatter(the1,the2,10,phi1c4,'filled')
xlim([0 180])
ylim([0 180])
xticks([0, 45, 90, 135, 180])
yticks([0, 45, 90, 135, 180])
axis square
grid on
grid minor
xlabel('\theta_1')
ylabel('\theta_2')
title('State 4: \phi_1 = 180')

cb = colorbar;
ylabel(cb,'\phi_i','FontSize',12,'FontWeight', 'bold','Rotation',360)
cb.Layout.Tile = 'south';

toc % Measure how long the program took to run