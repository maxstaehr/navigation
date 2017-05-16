classdef PositionQueue < handle
    %POSITIONQU Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        queue = zeros(4,4);
    end
    
    methods
        function obj = PositionQueue(pufferSize)
            queue = zeros(4,pufferSize);
        end
        
        function add(obj, pos, time)
            array = obj.queue(:,2:end);
            obj.queue = horzcat(array, [pos time]');
        end
        
        
        
        function posPast(obj, pos, time)
            
            x = interp1(obj.queue(4,:), obj.queue(1,:), time, 'nearest', 'extrap');
            y = interp1(obj.queue(4,:), obj.queue(2,:), time, 'nearest', 'extrap');
            theta = interp1(obj.queue(4,:), obj.queue(3,:), time, 'nearest', 'extrap');
            errorCorrection = egoKompensatePunkte([x y theta], [0 0 0],  pos);
            
            for i=1:size(obj.queue, 2)
                c = obj.queue(1:3,i);                
                pos = c';
                poscor  = forwardTransformation(pos, errorCorrection);
                obj.queue(1:3,i) = poscor';                
            end
            
        end
    end
    
end

