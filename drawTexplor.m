function [] = drawTexplor(Tsb,theta1,theta2,phi1,phi2)
% pre-transformation
r = 1;
% Origin is (0,0,0) and the arc is in xy plane
ptonlink3 = @(r,alpha) r*[cos(alpha);sin(alpha);zeros(size(alpha));...
    ones(size(alpha))];
plotarc = @(x) plot3(x(1,:),x(2,:),x(3,:),'Linewidth',5);
plotmass = @(x,color) plot3(x(1,:),x(2,:),x(3,:),'Marker','o','Linewidth',5,'MarkerSize',10, 'Color',color);
a1 = ptonlink3(r,0:0.1:pi);
p1 = ptonlink3(r,theta1);
q1 = ptonlink3(r,phi1);
T12 = [0,0,1,0;
    0,-1,0,0;
    1,0,0,0;
    0,0,0,1];
% keyboard
a2 = T12*a1;
p2 = T12*ptonlink3(r,theta2);
q2 = T12*ptonlink3(r,phi2);
plotarc(Tsb*a1);
hold on;
% keyboard
plotarc(Tsb*a2);
plotmass(Tsb*p1,'k');
plotmass(Tsb*p2,'k');
plotmass(Tsb*q1,'b');
plotmass(Tsb*q2,'b');
axis equal;
end

