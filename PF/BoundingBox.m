classdef BoundingBox
    %UNTITLED7 Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        H;
        xdim;
        ydim;
        zdim;
        intervall;
        
        coord;
    end
    
    methods
        function bb = BoundingBox(H, xdim, ydim, zdim)
            bb.H = H;
            bb.xdim = xdim;
            bb.ydim = ydim;
            bb.zdim = zdim;
            
            obj.coord = [0 0 0;
                bb.xdim 0 0;
                bb.xdim bb.ydim 0;
                0 bb.ydim 0;        
                0 0 bb.zdim;
                bb.xdim 0 bb.zdim;
                bb.xdim bb.ydim bb.zdim;
                0 bb.ydim bb.zdim];
            
        end
        
        function serialize(obj, fileId)
            for r=1:4
                for c=1:4
                  fwrite(fileId,  obj.H(r,c), 'single');          
                end
            end
            
            fwrite(fileId,  obj.xdim, 'single'); 
            fwrite(fileId,  obj.ydim, 'single');
            fwrite(fileId,  obj.zdim, 'single');
            
        end
        
        function ret = isInBox(bb, t)
            t4 = [t' 1]' ;          
            t4 = (bb.H\t4)';
         
            x = t4(1); 
            y = t4(2);
            z = t4(3);

            x0 =  isin(x,-bb.intervall,bb.xdim + bb.intervall);
            y0 =  isin(y,-bb.intervall,bb.ydim + bb.intervall);
            z0 =  isin(z,-bb.intervall,bb.zdim + bb.intervall);

            if(x0 && y0 && z0 )
               ret = true;
            else
               ret = false;
            end
        
        end
        
        function plotBox(bb, c)
        %plotBox plotes the bounding box in color c
        %   Detailed explanation goes here
        coord = [0 0 0;
                bb.xdim 0 0;
                bb.xdim bb.ydim 0;
                0 bb.ydim 0;        
                0 0 bb.zdim;
                bb.xdim 0 bb.zdim;
                bb.xdim bb.ydim bb.zdim;
                0 bb.ydim bb.zdim];


        for i=1:1:length(coord)
            p = [coord(i,:) 1];
            p = (bb.H*p')';
            p = p(1:1:3);
            coord(i,:) = p(:);
        end
        p1 = [
            coord(1,:);
            coord(4,:);
            coord(8,:);
            coord(5,:)
            ];

        p2 = [
            coord(2,:);
            coord(3,:);
            coord(7,:);
            coord(6,:)
            ];
        p3 = [
            coord(1,:);
            coord(5,:);
            coord(6,:);
            coord(2,:)
            ];
        p4 = [
            coord(3,:);
            coord(4,:);
            coord(8,:);
            coord(7,:)
            ];
        p5 = [
            coord(1,:);
            coord(2,:);
            coord(3,:);
            coord(4,:)
            ];

        p6 = [
            coord(5,:);
            coord(6,:);
            coord(7,:);
            coord(8,:)
            ];


            h = patch(p1(:,1), p1(:,2), p1(:,3),c);
            set(h, 'FaceAlpha',0.25);
            h = patch(p2(:,1), p2(:,2), p2(:,3),c);
            set(h, 'FaceAlpha',0.25);
            h = patch(p3(:,1), p3(:,2), p3(:,3),c);
            set(h, 'FaceAlpha',0.25);
            h = patch(p4(:,1), p4(:,2), p4(:,3),c);
            set(h, 'FaceAlpha',0.25);
            h = patch(p5(:,1), p5(:,2), p5(:,3),c);
            set(h, 'FaceAlpha',0.25);
            h = patch(p6(:,1), p6(:,2), p6(:,3),c);
            set(h, 'FaceAlpha',0.25);
        end
        
        function obj = transform(obj, H1)
            obj.H = H1*obj.H;
        end
        
    end
    
    
end

