classdef Robot
    %ROBOT Summary of this class goes here
    %   Detailed explanation goes here

    properties
        frontScanner;
        rearScanner;        
        frontScannerPosition = [ 0.5 0.5 -pi/2];
        rearScannerPosition = [ -0.5 -0.5 pi/2];
        scannerRes = [deg2rad(.5), 3*pi/2];
%         scannerRes = [10*deg2rad(.5), 3*pi/2];
        SH = [0.5 0.5;
                  0.5 -0.5;
                  -0.5 -0.5;            
                  -0.5 0.5
                  0.5 0.5]';
              
          NNeighbours = 5;
          NB;
          currentX = zeros(1,3);
          ANGLE = [];
          
          
          
    end
    
    methods
        function obj = Robot(superSamplingFaktor)
            obj.frontScanner = S300(obj.scannerRes(1), obj.scannerRes(2));            
            obj.rearScanner = S300(obj.scannerRes(1), obj.scannerRes(2));           
            
            da = obj.scannerRes(1)/superSamplingFaktor;
            

          obj.ANGLE = linspace(da, obj.scannerRes(1)-da, superSamplingFaktor);
          obj.ANGLE = obj.ANGLE - obj.scannerRes(1)/2;

        end
        
        function xr = getFrontLaserTransformation(obj)
            v1 = rot2(obj.currentX(3)) *obj.frontScannerPosition(1:2)';
            v2 = bsxfun(@plus, v1, obj.currentX(1:2));
            xr = [v2'  obj.currentX(3)+obj.frontScannerPosition(3)];   
        end
        
        function xr = getRearLaserTransformation(obj)
            v1 = rot2(obj.currentX(3)) *obj.rearScannerPosition(1:2)';
            v2 = bsxfun(@plus, v1, obj.currentX(1:2));
            xr = [v2'  obj.currentX(3)+obj.rearScannerPosition(3)];    
        end        
        
        function plot(obj, ax)
            axes(ax);
%             line(obj.SH(1,:), obj.SH(2,:), 'LineWidth', 25, 'Color', 'r');

            v1 = rot2(obj.currentX(3)) *obj.frontScannerPosition(1:2)';
            v2 = bsxfun(@plus, v1, obj.currentX(1:2));
            xr = [v2'  obj.currentX(3)+obj.frontScannerPosition(3)];                
            obj.frontScanner.plot(ax, xr);
            
            v1 = rot2(obj.currentX(3)) *obj.rearScannerPosition(1:2)';
            v2 = bsxfun(@plus, v1, obj.currentX(1:2));
            xr = [v2'  obj.currentX(3)+obj.rearScannerPosition(3)];                 
            obj.rearScanner.plot(ax, xr);
        end
        
        function plot_local(obj, ax)
%             obj.frontScanner.plot_local(ax);
%             obj.rearScanner.plot_local(ax);
        end
        function obj = transform(obj, rot, x)
            SH1 = rot2(rot)*obj.SH;
            obj.SH = bsxfun(@plus, SH1, x);       
            obj.currentX = [x; rot];
        end
        
        function obj=raytrace(obj, env)
            v1 = rot2(obj.currentX(3)) *obj.frontScannerPosition(1:2)';
            v2 = bsxfun(@plus, v1, obj.currentX(1:2));
            xr = [v2'  obj.currentX(3)+obj.frontScannerPosition(3)];            
            obj.frontScanner = obj.frontScanner.raytrace(env, xr);
            

            v1 = rot2(obj.currentX(3)) *obj.rearScannerPosition(1:2)';
            v2 = bsxfun(@plus, v1, obj.currentX(1:2));
            xr = [v2'  obj.currentX(3)+obj.rearScannerPosition(3)];                        
            obj.rearScanner = obj.rearScanner.raytrace(env, xr);
        end
        
        function PCL = generateGlobalPCL(robot)
            xf = robot.getFrontLaserTransformation();
            xr = robot.getRearLaserTransformation();
                  
            PCL = zeros(2*length(robot.ANGLE), 2);
            index = 1;
            At = robot.frontScanner.A + xf(3);
            for i=1:length(At)
                for j=1:length(robot.ANGLE)
                    XPOS = robot.frontScanner.D(i) *cos(At(i)+robot.ANGLE(j)) + xf(1);
                    YPOS = robot.frontScanner.D(i) *sin(At(i)+robot.ANGLE(j)) + xf(2);
                     PCL(index, :) = [XPOS YPOS];
                     index = index+1;
                end
            end
            At = robot.rearScanner.A + xr(3);
            for i=1:length(At)
                for j=1:length(robot.ANGLE)
                    XPOS = robot.rearScanner.D(i)*cos(At(i)+robot.ANGLE(j)) + xr(1);
                    YPOS = robot.rearScanner.D(i)*sin(At(i)+robot.ANGLE(j)) + xr(2);
                     PCL(index, :) = [XPOS YPOS];
                     index = index+1;
                end
            end

%             At = robot.rearScanner.A + xr(3);
%             XPOS = robot.rearScanner.D .*cos(At) + xr(1);
%             YPOS = robot.rearScanner.D .*sin(At) + xr(2);
%             PCL = vertcat(PCL, [XPOS' YPOS']);                 
        end
        
        


        function obj = initMinimumNeighbours(obj)
            %retrieve full cloud
            P = horzcat(obj.frontScanner.PCL_local, obj.rearScanner.PCL_local);
            N = size(P, 2);
            DIST = nan(size(P, 2));
            
            for row = 1:N
                for column = row:N
                    %dist = norm(P(:,row) - PCL(:,column));
                    vdist = P(:,row) - P(:,column);
                    dist = vdist(1)^2+vdist(2)^2;
                    DIST(row, column) = dist;
                    DIST(column, row) = dist;
                end
            end            
            %find minimum distance allong each colum
            obj.NB = zeros(size(P, 2), obj.NNeighbours);
            
            for row =1:N
                x = DIST(row,:);                               
                for i=1:obj.NNeighbours
                  [d, idx] = min(x);
                  % remove for the next iteration the last smallest value:
                  x(idx) = nan;
                  obj.NB(row,i) = idx;
                end
            end
           
        end
        
        function dist = calculateAverageDistance(obj, PCL)
            %retrieve full cloud
            P = horzcat(obj.frontScanner.PCL_local, obj.rearScanner.PCL_local);
            N = size(P, 2);
            
            x = zeros(1,obj.NNeighbours);    
            minV = zeros(size(P,2), 1);
            for row = 1:N                
                for column = 1:obj.NNeighbours
                    %dist = norm(P(:,row) - PCL(:,column));
                    vdist = P(:,row) - PCL(:,obj.NB(row,column));
                    x(column) = vdist(1)^2+vdist(2)^2;
                end
                minV(row) = min(x);
            end                               
            dist = sum(minV)/N;
        end
        
    end
    
end

