classdef ParticleFilter < handle
    %UNTITLED Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        N = 1e2;
        EST;
        PRED;
        dt = 0.01;
        q_trans = [1e-5 0 0];
        q_rot = [1e-1 0 0];
        
    end
    
    methods
        function obj = ParticleFilter()
            obj.EST = zeros(5,obj.N);
            obj.PRED = zeros(5,obj.N);
            
        end
        
        function predict(obj, transO, rotO)

            NTrans = horzcat(obj.q_trans(1)*(rand(size(obj.EST, 2), 1)-0.5), ...
                        obj.q_trans(1)*(rand(size(obj.EST, 2), 1)-0.5), ...
                        obj.q_trans(1)*(rand(size(obj.EST, 2), 1)-0.5));
            
            NRot = horzcat(obj.q_rot(1)*(rand(size(obj.EST, 2), 1)-0.5), ...
                        obj.q_rot(1)*(rand(size(obj.EST, 2), 1)-0.5), ...
                        obj.q_rot(1)*(rand(size(obj.EST, 2), 1)-0.5));
                    
            for i=1:size(obj.EST, 2)
                
                trans = transO + NTrans(i,:);
                rot = rotO + NRot(i,:);
                
                f = [1/6 1/2 1 0];
                p= fliplr([0 trans]).*f;                        
                dtrans = polyval(p,obj.dt);

                p = fliplr([0 rot]).*f;                        
                drot = polyval(p,obj.dt);

                f = [1/2 1 1];
                p = fliplr(trans).*f;                        
                vtrans = polyval(p,obj.dt);

                p = fliplr(rot).*f;                        
                vrot = polyval(p,obj.dt);                
                
                
                
                xpos =    obj.EST(1,i);
                ypos =    obj.EST(2,i);
                theta =    obj.EST(3,i);

                obj.PRED(1,i) = cos(theta + drot)*dtrans + xpos;
                obj.PRED(2,i) = sin(theta + drot)*dtrans + ypos;
                obj.PRED(3,i) = theta + drot;
                obj.PRED(4,i) = vtrans;
                obj.PRED(5,i) = vrot;
            end
                                    
        end
        
        function plotPaticle(obj, ax)
            plot(ax, obj.EST(1,:), obj.EST(2,:), 'xr');
            plot(ax, obj.PRED(1,:), obj.PRED(2,:), 'og');
            legend('EST', 'PRED');
        end
        
        function step(obj)
            obj.EST = obj.PRED;
        end
    end
    
end

