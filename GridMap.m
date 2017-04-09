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
        superSamplingFaktor;
        deltaAngle;
        XcoordInVoxel;
        YcoordInVoxel;
        WcoordInVoxel;
        
        hasData = false;
        
    end
    
    methods
        %function obj = GridMap(w, h, d, da)
        function obj = GridMap(xmin, xmax,ymin, ymax, d, da)
            obj.deltaAngle = da;
            w = ymax -ymin;
            h = xmax - xmin;
            maxrayLength = norm([w h]');
            minCellAngle = atan2(d,maxrayLength);
            obj.superSamplingFaktor = ceil(da/minCellAngle);
            
            
            

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
            
            obj.XcoordInVoxel = 0*obj.ws_x;
            obj.YcoordInVoxel= 0*obj.ws_x;
            obj.WcoordInVoxel= 0*obj.ws_x;
            
            
            
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
%             Z(Z==0) = nan;
            X = reshape(obj.ws_x, obj.ny,obj.nx);
            Y = reshape(obj.ws_y, obj.ny,obj.nx);
            Z = reshape(Z, obj.ny,obj.nx);

            surf(ax, X,Y,Z);           

            
%             Xoffset = reshape(obj.XcoordInVoxel, obj.ny,obj.nx);
%             Yoffset = reshape(obj.YcoordInVoxel, obj.ny,obj.nx);
%             
%             Xresult = Xoffset + X;
%             Yresult = Yoffset + Y;
%             Xresult(Z < 0.1) = nan;
%             Yresult(Z < 0.1) = nan;
%             
%             plot(ax, Xresult, Yresult, 'xm');
            
        end
        
        function [h h2] = plotWeightedVoxel(obj, ax)
            
            Z = zeros(1, obj.nx*obj.ny);
            for i=1:length(Z)
                if  obj.B(i) > 0
                    Z(i) = obj.B(i) / obj.V(i);
                end
            end
%             Z(Z==0) = nan;
            X = reshape(obj.ws_x, obj.ny,obj.nx);
            Y = reshape(obj.ws_y, obj.ny,obj.nx);
            Z = reshape(Z, obj.ny,obj.nx);            
            
            Xoffset = reshape(obj.XcoordInVoxel, obj.ny,obj.nx);
            Yoffset = reshape(obj.YcoordInVoxel, obj.ny,obj.nx);
            
            Xresult = Xoffset + X;
            Yresult = Yoffset + Y;
            Xresult(Z < 0.1) = nan;
            Yresult(Z < 0.1) = nan;
            X(Z < 0.1) = nan;
            Y(Z < 0.1) = nan;
            
            
            h = plot(ax, Xresult, Yresult, 'xb');
            h2 = plot(ax, X, Y, 'xk');
            h= h(1);
            h2 = h2(1);
        end
        
        function [xid, yid] = getID(obj, xpos, ypos)
            
            xid = round(((xpos-obj.xmin)/(obj.xmax-obj.xmin))*(obj.nx-1))+1;
            yid = round(((ypos-obj.ymin)/(obj.ymax-obj.ymin))*(obj.ny-1))+1;
        end
        
      function PCL = superSample(obj, o, PCLin)
          
          s = obj.superSamplingFaktor*size(PCLin, 1);
          PCL = zeros(s, 2);
          da = obj.deltaAngle/obj.superSamplingFaktor;
          ANGLE = linspace(da, obj.deltaAngle-da, obj.superSamplingFaktor);
          ANGLE = ANGLE - obj.deltaAngle/2;
          index = 1;
          for i=1:size(PCLin,1)
              dir = PCLin(i,:)'-o';
              for j=1:length(ANGLE)
                  t = rot2(ANGLE(j))*dir;
                  v = bsxfun(@plus, o', t);
                  PCL(index, :) = v';                 
                  index = index+1;
              end
          end
          
      end
        
        
      function obj = raytraceRayGridPCL(obj, o, PCL)
          
          if ~obj.hasData
              obj.hasData = true;              
          end
          
          PCL = obj.superSample(o,PCL);

            xid = round(((o(1)-obj.xmin)/(obj.xmax-obj.xmin))*(obj.nx-1))+1;
            yid = round(((o(2)-obj.ymin)/(obj.ymax-obj.ymin))*(obj.ny-1))+1;
            
            
            
            
            for ray=1:size(PCL,1)                                            

                
                xidend = round(((PCL(ray,1)-obj.xmin)/(obj.xmax-obj.xmin))*(obj.nx-1))+1;
                yidend = round(((PCL(ray,2)-obj.ymin)/(obj.ymax-obj.ymin))*(obj.ny-1))+1;                
                
                x = xid;
                y = yid;
                id = double((x-1)*obj.ny + (y-1) +1);
                
                xpos = o(1);
                ypos = o(2);
                
                endpoint = PCL(ray,:)';
                dir = PCL(ray,:)' - o';
                ndir = dir/norm(dir);
                
                rv1 = ndir(1);
                rv2 = ndir(2);
                
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
                        
%                         
%                         deltaX = endpoint(1)-obj.ws_x(id);
%                         obj.XcoordInVoxel(id) = (obj.XcoordInVoxel(id)*obj.WcoordInVoxel(id) + deltaX)/(obj.WcoordInVoxel(id)+1);
%                         
%                         deltaY = endpoint(2)-obj.ws_y(id);
%                         obj.YcoordInVoxel(id) = (obj.YcoordInVoxel(id)*obj.WcoordInVoxel(id) + deltaY)/(obj.WcoordInVoxel(id)+1);                        
%                         
% 
%                         obj.WcoordInVoxel(id)= obj.WcoordInVoxel(id)+1;
                        
                        break;
                    end


                end
            end
        end        
        
        
        function obj = raytraceRayGrid(obj, xpos, ypos, theta, A, D)
            
          if ~obj.hasData
              obj.hasData = true;              
          end

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
        
        
  	

        function SM = getSupMap(obj,xid, yid, nnx, nny, SM)
            xstart = max(1,xid -floor(nnx/2));
            ystart = max(1,yid -floor(nny/2));
            xend = min(obj.nx,xid +floor(nnx/2));
            yend = min(obj.ny,yid +floor(nny/2));            
            index = 1;
            for x=xstart:xend
                for y=ystart:yend
                    id = double((x-1)*obj.ny + (y-1) +1);           
                    SM(1,index)=  obj.ws_x(id)+obj.XcoordInVoxel(id); 
                    SM(2,index) = obj.ws_y(id)+obj.YcoordInVoxel(id); 
                    SM(3,index) = obj.B(id); 
                    SM(4,index)=  obj.V(id);
                    index = index +1;
                end
            end            
        end
        
    end
end

