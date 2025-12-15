% Plots phi values in relation to input theta angles

figure()
tcl = tiledlayout(1,2);

nexttile(tcl)
theta1 = [1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16 17 18 19 20 21 22 23 24 25];
phi1c1 = [0 145 135 145 180 60 70 160 180 70 90 90 90 90 90 135 180 25 115 135 180 35 45 35 165];
phi1c2 = [20 145 140 155 180 60 70 160 180 50 90 90 90 90 90 135 180 25 110 135 180 35 45 35 165];
phi1c1sim = [0 145 135 145 180 45 90 158 180 45 90 90 90 90 90 135 180 23 90 135 180 35 45 35 180];
% subplot(2,2,1)
plot(theta1,phi1c1,'r--x',theta1,phi1c2,'g:+',theta1,phi1c1sim, 'b--o','MarkerSize', 8, 'LineWidth',2);
ax = gca;
ax.FontSize = 15;
xticks([1 3 5 7 9 11 13 15 17 19 21 23 25])
xticklabels({'(0,0)','(0,90)','(0,180)','(45,45)','(45,135)','(90,0)','(90,90)','(90,180)','(135,45)','(135,135)','(180,0)','(180,90)','(180,180)'})
hold on
lgd2 = legend('State 1 Exp.','State 2 Exp.','States 1 and 2 Sim.','Location','southoutside', 'FontSize',15);
lgd2.NumColumns = 3;
xlabel('(\theta_1, \theta_2)','fontweight','bold', 'FontSize',15)
ylabel('\phi_1','fontweight','bold', 'FontSize',15)
title('State 1 where \phi_2 = 0 and State 2 where \phi_2 = 180', 'FontSize',15)
grid on
grid minor
% %view
% hold on
% 
% subplot(2,2,2)

% nexttile(tcl)
% phi1c2 = [20 145 140 155 180 60 70 160 180 50 90 90 90 90 90 135 180 25 110 135 180 35 45 35 165];
% phi1c2sim = [0 145 135 145 180 45 90 158 180 45 90 90 90 90 90 135 180 23 90 135 180 35 45 35 180];
% plot(theta1,phi1c2,'r--x',theta1,phi1c2sim, 'b--o');
% xticks([1 3 5 7 9 11 13 15 17 19 21 23 25])
% xticklabels({'(0,0)','(0,90)','(0,180)','(45,45)','(45,135)','(90,0)','(90,90)','(90,180)','(135,45)','(135,135)','(180,0)','(180,90)','(180,180)'})
% hold on
% % legend('Experimental','Simulated');
% xlabel('(\theta_1, \theta_2)')
% ylabel('\phi_1')
% title('Case 2 where \phi_2 = 180')
% 
% hold on
% 
% subplot(2,2,3)

nexttile(tcl)
phi1c3 = [0 45 90 130 180 145 75 90 180 35 140 160 90 25 45 150 180 90 110 35 180 45 90 135 160];
phi1c4 = [20 55 90 130 155 145 75 90 180 30 145 160 90 25 45 150 180 90 110 35 180 55 90 135 165];
phi1c3sim = [0 45 90 135 180 145 90 90 180 35 135 158 90 23 45 145 180 90 90 35 180 45 90 135 180];
plot(theta1,phi1c3,'m--x',theta1,phi1c4,'c:+',theta1,phi1c3sim, 'b--o','MarkerSize', 8, 'LineWidth',2);
ax2 = gca;
ax2.FontSize = 15;
xticks([1 3 5 7 9 11 13 15 17 19 21 23 25])
xticklabels({'(0,0)','(0,90)','(0,180)','(45,45)','(45,135)','(90,0)','(90,90)','(90,180)','(135,45)','(135,135)','(180,0)','(180,90)','(180,180)'})
hold on
lgd = legend('State 3 Exp.','State 4 Exp.','States 3 and 4 Sim.','Location','southoutside', 'FontSize',15);
lgd.NumColumns = 3;
xlabel('(\theta_1, \theta_2)','fontweight','bold','fontweight','bold', 'FontSize',15)
ylabel('\phi_2','fontweight','bold', 'FontSize',15)
title('State 3 where \phi_1 = 0 and State 4 where \phi_1 = 180', 'FontSize',15)
grid on
grid minor

% hold on
% 
% subplot(2,2,4)

% nexttile(tcl)
% phi1c4 = [20 55 90 130 155 145 75 90 180 30 145 160 90 25 45 150 180 90 110 35 180 55 90 135 165];
% phi1c4sim = [0 45 90 135 180 145 90 90 180 35 135 158 90 23 45 145 180 90 90 35 180 45 90 135 180];
% plot(theta1,phi1c4,'r--x',theta1,phi1c4sim, 'b--o');
% xticks([1 3 5 7 9 11 13 15 17 19 21 23 25])
% xticklabels({'(0,0)','(0,90)','(0,180)','(45,45)','(45,135)','(90,0)','(90,90)','(90,180)','(135,45)','(135,135)','(180,0)','(180,90)','(180,180)'})
% hold on
% % legend('Experimental','Simulated');
% xlabel('(\theta_1, \theta_2)')
% ylabel('\phi_2')
% title('Case 4 where \phi_1 = 180')

% hl = legend('Experimental for Cases 1 and 3','Experimental for Cases 2 and 4','Simulated');
% hl.Layout.Tile = 'South';
