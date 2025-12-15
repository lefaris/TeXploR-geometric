% Agile Robotics Laboratory  at UA
% TExploR Project
% Date: 07/27/2023
% 
% This class takes in theta1, theta2 values (one for each arc)
% and returns Tsb and zb.  The resulting 4 arc combinations are
% then plotted for visualization.
%
% This is achieved through the geometric relationship between the
% two arcs, the holonomic constraints that stick the  points of 
% contact to the ground plane at all times, and the statics.
%
% Usage:
% Create an instance of the TExploRStatics class with the desired
% theta1 and theta2 values:
% labCodeObj = TExploRStatics(0, 90);

% Compute and get the static poses and plots for cases 2 and 3:
% staticPoses = labCodeObj.computeStaticPoses();

% Use other methods and properties of the class as needed. For example,
% you can access the transformation matrix for case 2a as follows:
% case2_Tsb_1 = staticPoses.Case2(1).Tsb;

classdef TExploRStatics < handle
    
    properties
        T12
        R12
        gc
        m
        M
        rB
        Fr10
        Fr20
        zs
        theta1
        theta2
        staticPoses
        r
        tt
        increment
    end
    
    methods
        function obj = TExploRStatics(theta1_deg, theta2_deg) % Only need theta1, theta2 inputs
            obj.T12 = [0,0,1,0; 0,-1,0,0; 1, 0, 0, 0; 0, 0, 0, 1]; % Transformation matrix between the two arcs
            obj.R12 = obj.T12(1:3,1:3); % Rotation matrix between the two arcs
            obj.gc = -9.81; % Gravity constant
            obj.m = 0.4; % 200g shifting mass weight - may need to change (FIXME)
            obj.M = 0.5; % 300g arc weight - may need to change (FIXME)
            obj.rB = 0.15; % 0.15m distance from center - may need to change (FIXME)
            obj.Fr10 = (2*(obj.m*obj.gc) + 2*(obj.M*obj.gc))/2; % Reaction force on arc 1
            obj.Fr20 = obj.Fr10; % Reaction force on arc 2
            obj.zs = [0;0;1]; % zs hat
            obj.theta1 = deg2rad(theta1_deg); % Convert theta1 to radians
            obj.theta2 = deg2rad(theta2_deg); % Convert theta2 to radians
            obj.r = 1;
            obj.staticPoses = struct('Case2', [], 'Case3', []); % Create two cases of only t1b or t2b existing
            obj.tt = tiledlayout(2,2); % Start the 2x2 plotting grid for the two cases
            obj.increment = 1;
        end
        
        function ang = wrapTozero2Pi(obj, x) % Constrain between 0 and 2pi
            ang = (1 - sign(x)) * pi / 2 + x;
        end
        
        function p = ptonlink3(obj, r, alpha) % Calculate point on arc
            p = r * [cos(alpha), sin(alpha), 0]';
        end
        
        function t = ptonlinkTangent3(obj, r, alpha) % Calculate point on arc tangent
            t = r * [-sin(alpha), cos(alpha), 0]';
        end
        
        function R = getRMatnumeric(obj, zB, zs) % Calculate rotation matrix from zB to zs
            omega = LieGroup.vec2so3(zB)*zs; % Axis of rotation
            % theta12 = atan2(omegaNorm,v1'*v2);
            ctheta = zB'*zs/(norm(zB)*norm(zs));
            stheta = norm(omega);
            theta = atan2(stheta,ctheta);
            omega = omega/norm(omega);
            R = LieGroup.MatExponential3(omega,theta);
        end

        function staticPoses = computeStaticPoses(obj)
            % Case 2 (only t1b exists)
            % Case 2a phi2 = 0
            obj.staticPoses.Case2(1).phi1 = obj.wrapTozero2Pi(atan((sin(obj.theta1) - sin(obj.theta2)) / cos(obj.theta1)));
            obj.staticPoses.Case2(1).phi2 = 0;
            obj.staticPoses.Case2(1).zB = [cos(obj.staticPoses.Case2(1).phi1), sin(obj.staticPoses.Case2(1).phi1), 1]'/sqrt(2);

            % Testing plotting the first stage when phi1 is in 0-180 and
            % phi2 = 0
            %{
            figure(1)
            for k = 1:4
                if k == 1
                    the1 = 180;
                end
                if k == 2
                    the2 = 180;
                end
                if k == 3
                    the1 = 0;
                end
                if k == 4
                    the2 = 0;
                end
                if (k == 2)||(k == 4)
                    for j = 1:5:180
                        obj.staticPoses.Case2(3).phi1 = obj.wrapTozero2Pi(atan((sin(deg2rad(j)) - sin(deg2rad(the2))) / cos(deg2rad(j))));
                        obj.staticPoses.Case2(3).phi2 = 180;
                        obj.staticPoses.Case2(3).zB = [cos(obj.staticPoses.Case2(3).phi1), sin(obj.staticPoses.Case2(3).phi1), 1]'/sqrt(2);
                        obj.staticPoses.Case2(3).qB1 = obj.ptonlink3(obj.r, obj.staticPoses.Case2(3).phi1);
                        obj.staticPoses.Case2(3).Rsb = obj.getRMatnumeric(obj.staticPoses.Case2(3).zB, obj.zs);
                        obj.staticPoses.Case2(3).psb = [0; 0; -obj.staticPoses.Case2(3).zB' * obj.staticPoses.Case2(3).qB1];
                        obj.staticPoses.Case2(3).Tsb = [obj.staticPoses.Case2(3).Rsb, obj.staticPoses.Case2(3).psb; 0, 0, 0, 1];
                
                        obj.quasiStatic(obj.staticPoses.Case2(3).Tsb,deg2rad(j),deg2rad(the2),obj.staticPoses.Case2(3).phi1,obj.staticPoses.Case2(3).phi2, k);
                    end
                end
                if (k == 1)||(k == 3)
                    for j = 1:5:180
                        obj.staticPoses.Case2(3).phi1 = 180;
                        obj.staticPoses.Case2(3).phi2 = obj.wrapTozero2Pi(atan((sin(deg2rad(the1)) - sin(deg2rad(j))) / cos(deg2rad(the1))));
                        obj.staticPoses.Case2(3).zB = [cos(obj.staticPoses.Case2(3).phi1), sin(obj.staticPoses.Case2(3).phi1), 1]'/sqrt(2);
                        obj.staticPoses.Case2(3).qB1 = obj.ptonlink3(obj.r, obj.staticPoses.Case2(3).phi1);
                        obj.staticPoses.Case2(3).Rsb = obj.getRMatnumeric(obj.staticPoses.Case2(3).zB, obj.zs);
                        obj.staticPoses.Case2(3).psb = [0; 0; -obj.staticPoses.Case2(3).zB' * obj.staticPoses.Case2(3).qB1];
                        obj.staticPoses.Case2(3).Tsb = [obj.staticPoses.Case2(3).Rsb, obj.staticPoses.Case2(3).psb; 0, 0, 0, 1];
                
                        obj.quasiStatic(obj.staticPoses.Case2(3).Tsb,deg2rad(the1),deg2rad(j),obj.staticPoses.Case2(3).phi1,obj.staticPoses.Case2(3).phi2, k);
                    end
                end
                hold on;
            end
            %}
            
            % Case 2b phi2 = pi
            obj.staticPoses.Case2(2).phi1 = obj.wrapTozero2Pi(atan((sin(obj.theta1) - sin(obj.theta2)) / cos(obj.theta1)));
            obj.staticPoses.Case2(2).phi2 = pi;
            obj.staticPoses.Case2(2).zB = [-cos(obj.staticPoses.Case2(2).phi1), -sin(obj.staticPoses.Case2(2).phi1), 1]'/sqrt(2);

            %HERE IS THE FORCE VALUES CHECK:
            obj.staticPoses.Case2(2).Fr1 = ((obj.M*obj.gc*sin(obj.staticPoses.Case2(2).phi1)) + (obj.M*obj.gc*sin(obj.theta1)) - (obj.M*obj.gc*sin(obj.theta2)) + (obj.m*obj.gc*sin(obj.staticPoses.Case2(2).phi1))...
                                            + (obj.M*obj.gc*cos(obj.theta2)*sin(obj.staticPoses.Case2(2).phi1)))/sin(obj.staticPoses.Case2(2).phi1);
            obj.staticPoses.Case2(2).Fr2 = (2^(1/2)*(2^(1/2)*obj.m*obj.gc*sin(obj.staticPoses.Case2(2).phi1) + 2^(1/2)*obj.M*obj.gc*sin(obj.staticPoses.Case2(2).phi1)...
                                            - 2^(1/2)*obj.M*obj.gc*sin(obj.theta1) + 2^(1/2)*obj.M*obj.gc*sin(obj.theta2) - 2^(1/2)*obj.M*obj.gc*cos(obj.theta2)*sin(obj.staticPoses.Case2(2).phi1)))/(2*sin(obj.staticPoses.Case2(2).phi1));


            
            % Testing plotting the first stage when phi1 is in 0-180 and
            % phi2 = 0
            figure(1)
            for k = 1:4
                if (k == 1)
                    for j = 1:5:180
                        obj.staticPoses.Case2(3).the1 = deg2rad(180);
                        obj.staticPoses.Case2(3).the2 = deg2rad(j);
                        obj.staticPoses.Case2(3).phi1 = 180;
                        obj.staticPoses.Case2(3).phi2 = obj.wrapTozero2Pi(atan((sin(obj.staticPoses.Case2(3).the2) - sin(obj.staticPoses.Case2(3).the1)) / cos(obj.staticPoses.Case2(3).the2)));;
                        %obj.staticPoses.Case2(3).the2 = obj.wrapTozero2Pi(atan((sin(deg2rad(180)) - sin(deg2rad(j))) / cos(deg2rad(180))));
                        obj.staticPoses.Case2(3).zB = [-1, sin(obj.staticPoses.Case2(3).phi2), -cos(obj.staticPoses.Case2(3).phi2)]'/sqrt(2);
                        obj.staticPoses.Case2(3).qB1 = obj.ptonlink3(obj.r, obj.staticPoses.Case2(3).phi1);
                        obj.staticPoses.Case2(3).Rsb = obj.getRMatnumeric(obj.staticPoses.Case2(3).zB, obj.zs);
                        obj.staticPoses.Case2(3).psb = [0; 0; -obj.staticPoses.Case2(3).zB' * obj.staticPoses.Case2(3).qB1];
                        obj.staticPoses.Case2(3).Tsb = [obj.staticPoses.Case2(3).Rsb, obj.staticPoses.Case2(3).psb; 0, 0, 0, 1];
                        obj.quasiStatic(obj.staticPoses.Case2(3).Tsb,obj.staticPoses.Case2(3).the1,obj.staticPoses.Case2(3).the2,obj.staticPoses.Case2(3).phi1,obj.staticPoses.Case2(3).phi2, k);
                    end
                end
                if (k == 2) %There's a problem here
                    for j = 180:-5:1
                        obj.staticPoses.Case2(3).the1 = deg2rad(j);
                        obj.staticPoses.Case2(3).the2 = deg2rad(180);
                        obj.staticPoses.Case2(3).phi1 = j;
                        obj.staticPoses.Case2(3).phi2 = obj.wrapTozero2Pi(atan((sin(obj.staticPoses.Case2(3).the2) - sin(obj.staticPoses.Case2(3).the1)) / cos(obj.staticPoses.Case2(3).the2)));;
                        %obj.staticPoses.Case2(3).the1 = obj.wrapTozero2Pi(atan((sin(deg2rad(j)) - sin(deg2rad(180))) / cos(deg2rad(j))));
                        obj.staticPoses.Case2(3).zB = [-1, sin(obj.staticPoses.Case2(3).phi2), -cos(obj.staticPoses.Case2(3).phi2)]'/sqrt(2);
                        obj.staticPoses.Case2(3).qB1 = obj.ptonlink3(obj.r, obj.staticPoses.Case2(3).phi1);
                        obj.staticPoses.Case2(3).Rsb = obj.getRMatnumeric(obj.staticPoses.Case2(3).zB, obj.zs);
                        obj.staticPoses.Case2(3).psb = [0; 0; -obj.staticPoses.Case2(3).zB' * obj.staticPoses.Case2(3).qB1];
                        obj.staticPoses.Case2(3).Tsb = [obj.staticPoses.Case2(3).Rsb, obj.staticPoses.Case2(3).psb; 0, 0, 0, 1];
                        obj.quasiStatic(obj.staticPoses.Case2(3).Tsb,obj.staticPoses.Case2(3).the1,obj.staticPoses.Case2(3).the2,obj.staticPoses.Case2(3).phi1,obj.staticPoses.Case2(3).phi2, k);
                    end
                end
                if (k == 3)
                    for j = 180:-5:1
                        obj.staticPoses.Case2(3).the1 = deg2rad(0);
                        obj.staticPoses.Case2(3).the2 = deg2rad(j);
                        obj.staticPoses.Case2(3).phi1 = 0;
                        obj.staticPoses.Case2(3).phi2 = obj.wrapTozero2Pi(atan((sin(obj.staticPoses.Case2(3).the2) - sin(obj.staticPoses.Case2(3).the1)) / cos(obj.staticPoses.Case2(3).the2)));;
                        %obj.staticPoses.Case2(3).the2 = obj.wrapTozero2Pi(atan((sin(deg2rad(0)) - sin(deg2rad(j))) / cos(deg2rad(0))));
                        obj.staticPoses.Case2(3).zB = [-1, sin(obj.staticPoses.Case2(3).phi2), -cos(obj.staticPoses.Case2(3).phi2)]'/sqrt(2);
                        obj.staticPoses.Case2(3).qB1 = obj.ptonlink3(obj.r, obj.staticPoses.Case2(3).phi1);
                        obj.staticPoses.Case2(3).Rsb = obj.getRMatnumeric(obj.staticPoses.Case2(3).zB, obj.zs);
                        obj.staticPoses.Case2(3).psb = [0; 0; -obj.staticPoses.Case2(3).zB' * obj.staticPoses.Case2(3).qB1];
                        obj.staticPoses.Case2(3).Tsb = [obj.staticPoses.Case2(3).Rsb, obj.staticPoses.Case2(3).psb; 0, 0, 0, 1];
                        obj.quasiStatic(obj.staticPoses.Case2(3).Tsb,obj.staticPoses.Case2(3).the1,obj.staticPoses.Case2(3).the2,obj.staticPoses.Case2(3).phi1,obj.staticPoses.Case2(3).phi2, k);
                    end
                end
                if (k == 4) %There's a problem here
                    for j = 1:5:180
                        obj.staticPoses.Case2(3).the1 = deg2rad(j);
                        obj.staticPoses.Case2(3).the2 = deg2rad(0);
                        obj.staticPoses.Case2(3).phi1 = j;
                        obj.staticPoses.Case2(3).phi2 = obj.wrapTozero2Pi(atan((sin(obj.staticPoses.Case2(3).the2) - sin(obj.staticPoses.Case2(3).the1)) / cos(obj.staticPoses.Case2(3).the2)));;
                        %obj.staticPoses.Case2(3).the1 = obj.wrapTozero2Pi(atan((sin(deg2rad(j)) - sin(deg2rad(0))) / cos(deg2rad(j))));
                        %Previously using the below zB calculation for all
                        %four cases!
                        %obj.staticPoses.Case2(3).zB = [cos(obj.staticPoses.Case2(3).phi2), sin(obj.staticPoses.Case2(3).phi2), 1]'/sqrt(2);
                        obj.staticPoses.Case2(3).zB = [-1, sin(obj.staticPoses.Case2(3).phi2), -cos(obj.staticPoses.Case2(3).phi2)]'/sqrt(2);
                        obj.staticPoses.Case2(3).qB1 = obj.ptonlink3(obj.r, obj.staticPoses.Case2(3).phi1);
                        obj.staticPoses.Case2(3).Rsb = obj.getRMatnumeric(obj.staticPoses.Case2(3).zB, obj.zs);
                        obj.staticPoses.Case2(3).psb = [0; 0; -obj.staticPoses.Case2(3).zB' * obj.staticPoses.Case2(3).qB1];
                        obj.staticPoses.Case2(3).Tsb = [obj.staticPoses.Case2(3).Rsb, obj.staticPoses.Case2(3).psb; 0, 0, 0, 1];
                        obj.quasiStatic(obj.staticPoses.Case2(3).Tsb,obj.staticPoses.Case2(3).the1,obj.staticPoses.Case2(3).the2,obj.staticPoses.Case2(3).phi1,obj.staticPoses.Case2(3).phi2, k);
                    end
                end
                hold on;
            end
            
            
            
            figure(2)
            for ii = 1:2
                % Calculate qb1, qb2, and Tsb for two sets of phi1 and phi2 values
                obj.staticPoses.Case2(ii).qB1 = obj.ptonlink3(obj.r, obj.staticPoses.Case2(ii).phi1);
                obj.staticPoses.Case2(ii).qB2 = obj.R12 * obj.ptonlink3(obj.r, obj.staticPoses.Case2(ii).phi2);
                obj.staticPoses.Case2(ii).Rsb = obj.getRMatnumeric(obj.staticPoses.Case2(ii).zB, obj.zs);
                obj.staticPoses.Case2(ii).psb = [0; 0; -obj.staticPoses.Case2(ii).zB' * obj.staticPoses.Case2(ii).qB1];
                obj.staticPoses.Case2(ii).Tsb = [obj.staticPoses.Case2(ii).Rsb, obj.staticPoses.Case2(ii).psb; 0, 0, 0, 1];
                % Sanity check
                obj.staticPoses.Case2(ii).holonomicCond = obj.staticPoses.Case2(ii).zB' * (obj.staticPoses.Case2(ii).qB1 - obj.staticPoses.Case2(ii).qB2);
                
                % Check for negative tangent value
                obj.staticPoses.Case2(ii).tanphi = obj.wrapTozero2Pi((sin(obj.theta1) - sin(obj.theta2)) / cos(obj.theta1));
                %obj.staticPoses.Case2(ii).tanphi = obj.wrapTozero2Pi((sin(obj.theta1) - sin(obj.theta2)) / cos(obj.theta1));

              
                % Plot case 2a and 2b
                nexttile()
                obj.drawTexplor(obj.staticPoses.Case2(ii).Tsb,obj.theta1,obj.theta2,obj.staticPoses.Case2(ii).phi1,obj.staticPoses.Case2(ii).phi2, obj.staticPoses.Case2(ii).zB);
                title(['Case2-',num2str(ii),', \phi_1=',num2str(rad2deg(obj.staticPoses.Case2(ii).phi1)) char(176),', \phi_2=',num2str(rad2deg(obj.staticPoses.Case2(ii).phi2)) char(176),', F_{R1}=',num2str(obj.staticPoses.Case2(2).Fr1),', F_{R2}=',num2str(obj.staticPoses.Case2(2).Fr2)]);
            end

            % Case 3 (only t2b exists)
            % Case 3a phi1 = 0
            obj.staticPoses.Case3(1).phi1 = 0;
            obj.staticPoses.Case3(1).phi2 = obj.wrapTozero2Pi(atan((sin(obj.theta2) - sin(obj.theta1)) / cos(obj.theta2)));
            obj.staticPoses.Case3(1).zB = [-1, sin(obj.staticPoses.Case3(1).phi2), -cos(obj.staticPoses.Case3(1).phi2)]'/sqrt(2);

            %HERE ARE THE FORCE VALUES CHECK:
            obj.staticPoses.Case3(1).Fr1 = ((obj.M*obj.gc*sin(obj.staticPoses.Case3(1).phi2)) + (obj.M*obj.gc*sin(obj.theta1)) - (obj.M*obj.gc*sin(obj.theta2)) + (obj.m*obj.gc*sin(obj.staticPoses.Case3(1).phi2))...
                                            + (obj.M*obj.gc*cos(obj.theta1)*sin(obj.staticPoses.Case3(1).phi2)))/sin(obj.staticPoses.Case3(1).phi2);
            obj.staticPoses.Case3(1).Fr2 = (2^(1/2)*(2^(1/2)*obj.m*obj.gc*sin(obj.staticPoses.Case3(1).phi2) + 2^(1/2)*obj.M*obj.gc*sin(obj.staticPoses.Case3(1).phi2)...
                                            - 2^(1/2)*obj.M*obj.gc*sin(obj.theta1) + 2^(1/2)*obj.M*obj.gc*sin(obj.theta2) - 2^(1/2)*obj.M*obj.gc*cos(obj.theta1)*sin(obj.staticPoses.Case3(1).phi2)))/(2*sin(obj.staticPoses.Case3(1).phi2));

            % Case 3b phi1 = pi
            obj.staticPoses.Case3(2).phi1 = pi;
            obj.staticPoses.Case3(2).phi2 = obj.wrapTozero2Pi(atan((sin(obj.theta2) - sin(obj.theta1)) / cos(obj.theta2)));
            obj.staticPoses.Case3(2).zB = [-1, -sin(obj.staticPoses.Case3(1).phi2), cos(obj.staticPoses.Case3(1).phi2)]'/sqrt(2);

            for ii = 1:2
                % Calculate qb1, qb2, and Tsb for two sets of phi1 and phi2 values
                obj.staticPoses.Case3(ii).qB1 = obj.ptonlink3(obj.r, obj.staticPoses.Case3(ii).phi1);
                obj.staticPoses.Case3(ii).qB2 = obj.R12 * obj.ptonlink3(obj.r, obj.staticPoses.Case3(ii).phi2);
                obj.staticPoses.Case3(ii).Rsb = obj.getRMatnumeric(obj.staticPoses.Case3(ii).zB, obj.zs);
                obj.staticPoses.Case3(ii).psb = [0; 0; -obj.staticPoses.Case3(1).zB' * obj.staticPoses.Case3(ii).qB1];
                obj.staticPoses.Case3(ii).Tsb = [obj.staticPoses.Case3(ii).Rsb, obj.staticPoses.Case3(ii).psb; 0, 0, 0, 1];
                % Sanity check
                obj.staticPoses.Case3(ii).holonomicCond = obj.staticPoses.Case3(ii).zB' * (obj.staticPoses.Case3(ii).qB1 - obj.staticPoses.Case3(ii).qB2);
                
                % Check for negative tangent value
                obj.staticPoses.Case3(ii).tanphi = obj.wrapTozero2Pi((sin(obj.theta2) - sin(obj.theta1)) / cos(obj.theta2));
                %obj.staticPoses.Case3(ii).tanphi = obj.wrapTozero2Pi((sin(obj.theta2) - sin(obj.theta1)) / cos(obj.theta2));

                % Plot case 3a and 3b
                nexttile()
                obj.drawTexplor(obj.staticPoses.Case3(ii).Tsb,obj.theta1,obj.theta2,obj.staticPoses.Case3(ii).phi1,obj.staticPoses.Case3(ii).phi2, obj.staticPoses.Case3(ii).zB);
                title(['Case3-',num2str(ii),', \phi_1=',num2str(rad2deg(obj.staticPoses.Case3(ii).phi1)) char(176),', \phi_2=',num2str(rad2deg(obj.staticPoses.Case3(ii).phi2)) char(176),', F_{R1}=',num2str(obj.staticPoses.Case3(1).Fr1),', F_{R2}=',num2str(obj.staticPoses.Case3(1).Fr2)]);
            end

            staticPoses = obj.staticPoses;
            obj.tt.Title.String=['\theta_1=',num2str(rad2deg(obj.theta1)),'deg, \theta_2=',num2str(rad2deg(obj.theta2)),'deg'];
        end
        
        function drawTexplor(obj, Tsb, theta1, theta2, phi1, phi2, zB)
            % Pre-transformation
            % Origin is (0,0,0) and the arc is in xy plane
            ptonlink3 = @(r,alpha) [r*cos(alpha);r*sin(alpha);zeros(size(alpha));ones(size(alpha))];
            %ptonlinkTangent3 = @(r,alpha) [r*-sin(alpha); r*cos(alpha); 0; 0];
            plotarc = @(x,color) plot3(x(1,:),x(2,:),x(3,:),'Linewidth',5,'Color',color);
            plotmassp = @(x,color) plot3(x(1,:),x(2,:),x(3,:),'Marker','o','Linewidth',5,'MarkerSize',10,'Color',color);
            plotmassq = @(x,color) plot3(x(1,:),x(2,:),x(3,:),'Marker','o','Linewidth',5,'MarkerSize',5,'Color',color);
            
            
            % Calculate arcs, mass positions, and points of contact
            a1 = ptonlink3(obj.r,0:0.1:pi);
            a2 = obj.T12*a1;
            p1 = ptonlink3(obj.r,theta1);
            p2 = obj.T12*ptonlink3(obj.r,theta2);
            q1 = ptonlink3(obj.r,phi1);
            q2 = obj.T12*ptonlink3(obj.r,phi2);
            %Trying out new rCOM
            %rCOM = Tsb*[0,0,0,1]';
            rCOM = Tsb*(((p1*obj.m)+(p2*obj.m)+ (a1*obj.M)+(a2*obj.M))/((2*obj.m) + (2*obj.M)));
            
            % Plot the arcs
            plotarc(Tsb*a1,'r');
            hold on;
            plotarc(Tsb*a2,'r');
            
            % Plot the masses and the point of contacts
            plotmassp(Tsb*p1,'m');
            plotmassp(Tsb*p2,'m');
            plotmassq(Tsb*q1,'b');
            plotmassq(Tsb*q2,'b');
            plot3(rCOM(1),rCOM(2),rCOM(3),'Marker','x','Linewidth',5,'MarkerSize',20,'Color','k');
            axis equal;
            xground = [2 2 -2 -2];
            yground = [2 -2 -2 2];
            c = [0.8 0.7 0.8];
            fill(xground,yground,c,'FaceAlpha',0.8);
        end
        
        function quasiStatic(obj, Tsb, theta1, theta2, phi1, phi2, col)
            % Pre-transformation
            % Origin is (0,0,0) and the arc is in xy plane
            
            ptonlink3 = @(r,alpha) [r*cos(alpha);r*sin(alpha);zeros(size(alpha));ones(size(alpha))];
            %ptonlinkTangent3 = @(r,alpha) [r*-sin(alpha); r*cos(alpha); 0; 0];
            plotarc1 = @(x,color) plot3(x(1,:)+(obj.increment/20),x(2,:),x(3,:),'Linewidth',3,'Color',color);
            plotarc2 = @(x,color) plot3(x(1,:)+(obj.increment/20),x(2,:),x(3,:),'Linewidth',3,'Color',color);
            plotmassp = @(x,color) plot3(x(1,:)+(obj.increment/20),x(2,:),x(3,:),'Marker','o','Linewidth',5,'MarkerSize',10,'Color',color);
            plotmassq = @(x,color) plot3(x(1,:)+(obj.increment/20),x(2,:),x(3,:),'Marker','o','Linewidth',5,'MarkerSize',5,'Color',color);
            plotCoM = @(x,color) plot3(x(1)+(obj.increment/20),x(2),x(3),'Marker','x','Linewidth',5,'MarkerSize',20,'Color',color);
            obj.increment = obj.increment + 1;
            % Calculate arcs, mass positions, and points of contact
            a1 = ptonlink3(obj.r,0:0.1:pi);
            a2 = obj.T12*a1;
            p1 = ptonlink3(obj.r,theta1);
            p2 = obj.T12*ptonlink3(obj.r,theta2);
            q1 = ptonlink3(obj.r,phi1);
            q2 = obj.T12*ptonlink3(obj.r,phi2);
            

            %Trying out new rCOM
            %rCOM = Tsb*[0,0,0,1]';
            rCOM = Tsb*(((p1*obj.m)+(p2*obj.m)+ (a1*obj.M)+(a2*obj.M))/((2*obj.m) + (2*obj.M)));


            if col == 1
                color = 'm';
            end
            if col == 2
                color = 'b';
            end
            if col == 3
                color = 'g';
            end
            if col == 4
                color = 'r';
            end
            % Plot the arcs
            plotarc1(Tsb*a1,color);
            hold on;
            plotarc2(Tsb*a2,[0 1 0]);
            
            % Plot the masses and the point of contacts
            plotmassp(Tsb*p1,'m');
            plotmassp(Tsb*p2,'m');
            plotmassq(Tsb*q1,'b');
            plotmassq(Tsb*q2,'b');
            plotCoM(rCOM,'k');
            axis equal;
            %xground = [2 2 -2 -2];
            %yground = [2 -2 -2 2];
            %c = [0.8 0.7 0.8];
            %fill(xground,yground,c,'FaceAlpha',0.8);
        end
    end
end
