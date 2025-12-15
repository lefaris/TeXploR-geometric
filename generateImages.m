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
myFolder = 'C:\Users\lefaris.STUDENT\Box\Agile Robotics Lab\TeXploreR\Experiments\static_angle_simulation_figures_V3';

% Adding theta1, theta2 for diagonal line
theta1 = [0 10 20 30 40 50 60 70 80 90 100 110 120 130 135];
theta2 = [0 3 7 10 13 16 20 23 26 30 33 36 40 43 45];

% Initialize theta and phi vectors to save memory
% the1 = zeros(8000);
% the2 = zeros(8000);
% phi1c1 = zeros(8000);
% phi1c2 = zeros(8000);
% phi1c3 = zeros(8000);
% phi1c4 = zeros(8000);

for i = 1:1:15 % For theta1 values - changing from 0:2:180
        TExploRStaticsDerivations(theta1(i),theta2(i)) % Run TExploreR sim code for each theta combo
        fname = sprintf('%d.png', k);
        fullName = fullfile(myFolder,fname);
        saveas(gcf,fullName);
        k = k+1;
end