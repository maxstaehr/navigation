classdef GridMap
    %MAP Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        width;
        height;
        d;
        B;
        V;
        nx;
        ny;
        xmin;
        xmax;
        ymin;
        ymax;
        ws_x;
        ws_y;
        xvoxelwidth;
        yvoxelwidth;
        
    end
    
    methods
        %function obj = GridMap(w, h, d, da)
        function obj = GridMap(xmin, xmax,ymin, ymax, d, da)
%             maxrayLength = norm([w h]');
%             minCellAngle = atan2(d,maxrayLength);
%             if minCellAngle > da
%                 while minCellAngle > da
%                     d = d/2;
%                     minCellAngle = atan2(d,maxrayLength);
%                 end                                
%             end              
            

            obj.width = xmax-xmin;
            obj.height = ymax-ymin;
            obj.nx = floor(obj.width/d)+1;
            obj.ny = floor(obj.height/d)+1;
            
            obj.xmin = xmin;
            obj.xmax = xmax;
            
            obj.ymin = ymin;
            obj.ymax = ymax;
            
            obj.xvoxelwidth = (obj.xmax -obj.xmin)/(obj.nx-1);
            obj.yvoxelwidth = (obj.ymax -obj.ymin)/(obj.ny-1);
            
            [X, Y] = meshgrid(obj.xmin: obj.xvoxelwidth:obj.xmax, obj.ymin:obj.yvoxelwidth:obj.ymax);
            obj.ws_x = reshape(X,1,numel(X));
            obj.ws_y = reshape(Y,1,numel(Y));
            
            
            
            obj.B = zeros(obj.ny, obj.nx);
            obj.V = zeros(obj.ny, obj.nx);
        end
        
        function updateMap(obj, xo, yo, theta, A, D)
            At = A + theta;

            xvoxelwidth = obj.d;
            yvoxelwidth = obj.d;
            

            
        end
        
        function obj = plotMap(obj, ax)
            axes(ax);
            X = [-obj.xvoxelwidth/2 obj.xvoxelwidth/2 obj.xvoxelwidth/2 -obj.xvoxelwidth/2];
            Y = [-obj.xvoxelwidth/2 -obj.xvoxelwidth/2 obj.xvoxelwidth/2 obj.xvoxelwidth/2];
            for i=1:obj.nx*obj.ny           
                line(X+obj.ws_x(i),Y+obj.ws_y(i));
            end
            
        end        
        
        function plotOccupancy(obj, ax)            
            for i=1:obj.nx*obj.ny
                if obj.V(i) > 0
                    plot(ax, obj.ws_x(i), obj.ws_y(i),  '.r');
                end
                if obj.B(i) > 0
                    plot(ax, obj.ws_x(i), obj.ws_y(i),  'om');
                end                
            end                                    
        end
        
        function plotProbabilityMap(obj, ax)
            Z = zeros(1, obj.nx*obj.ny);
            for i=1:length(Z)
                if  obj.B(i) > 0
                    Z(i) = obj.B(i) / obj.V(i);
                end
            end
            
            X = reshape(obj.ws_x, obj.ny,obj.nx);
            Y = reshape(obj.ws_y, obj.ny,obj.nx);
            Z = reshape(Z, obj.ny,obj.nx);
            surf(ax, X,Y,Z);           
        end
        
        function [xid, yid] = getID(obj, xpos, ypos)
            
            xid = round(((xpos-obj.xmin)/(obj.xmax-obj.xmin))*(obj.nx-1))+1;
            yid = round(((ypos-obj.ymin)/(obj.ymax-obj.ymin))*(obj.ny-1))+1;
        end
        
        function obj = raytraceRayGrid(obj, xpos, ypos, theta, A, D)

            xid = round(((xpos-obj.xmin)/(obj.xmax-obj.xmin))*(obj.nx-1))+1;
            yid = round(((ypos-obj.ymin)/(obj.ymax-obj.ymin))*(obj.ny-1))+1;
            
            
            xidend = round(((xpos-obj.xmin)/(obj.xmax-obj.xmin))*(obj.nx-1))+1;
            yidend = round(((ypos-obj.ymin)/(obj.ymax-obj.ymin))*(obj.ny-1))+1;
            
            At = A + theta;

            s = [xpos ypos]';
            for ray=1:length(At)                                            
                dm = D(ray);
                
       
                xpos1 = dm*cos(At(ray))+xpos;
                ypos1 = dm*sin(At(ray))+ypos;
                
                xidend = round(((xpos1-obj.xmin)/(obj.xmax-obj.xmin))*(obj.nx-1))+1;
                yidend = round(((ypos1-obj.ymin)/(obj.ymax-obj.ymin))*(obj.ny-1))+1;                
                
                x = xid;
                y = yid;
                id = double((x-1)*obj.ny + (y-1) +1);
                
                rv1 = cos(At(ray));
                rv2 = sin(At(ray));
                
                %setting endpoint as occupied
                
        
                if(rv1 < 0 )
                    stepX = -1;
                    tMaxX = abs((obj.ws_x(id) - 0.5*obj.xvoxelwidth - xpos)/rv1);
                else
                    stepX = 1;
                    tMaxX = abs((obj.ws_x(id) + 0.5*obj.xvoxelwidth - xpos)/rv1);
                end

                if(rv2 < 0 )
                    stepY = -1;
                    tMaxY = abs((obj.ws_y(id) - 0.5*obj.yvoxelwidth - ypos)/rv2);
                else
                    stepY = 1;
                    tMaxY = abs((obj.ws_y(id) + 0.5*obj.yvoxelwidth - ypos)/rv2);
                end
                
                tDeltaX = abs(obj.xvoxelwidth/rv1);
                tDeltaY = abs(obj.yvoxelwidth/rv2);
       
                while 1

                    if(tMaxX < tMaxY)                        
                        x = x +stepX;
                        if(x < 1 || x > obj.nx)
                            break;
                        end
                        tMaxX = tMaxX + tDeltaX;
                    else                        
                        y = y + stepY;
                        if(y < 1 || y > obj.ny)
                            break;
                        end
                        tMaxY = tMaxY + tDeltaY;                        

                    end


                    id = double((x-1)*obj.ny + (y-1) +1);                    
                    obj.V(id) = obj.V(id)+ 1;                                        
                    if xidend == x && yidend == y
                        obj.B(id) = obj.B(id) + 1;
                        break;
                    end


                end
            end
        end
        
        
  	

        function SM = getSupMap(obj,xid, yid, nnx, nny)
            xstart = max(1,xid -floor(nnx/2));
            ystart = max(1,yid -floor(nny/2));
            xend = min(obj.nx,xid +floor(nnx/2));
            yend = min(obj.ny,yid +floor(nny/2));
            SM = [];
            for x=xstart:xend
                for y=ystart:yend
                    id = double((x-1)*obj.ny + (y-1) +1);           
                    SM = horzcat(SM , [obj.ws_x(id); 
                        obj.ws_y(id); 
                        obj.B(id); 
                        obj.V(id)]);
                end
            end            
        end
        
    end
end

