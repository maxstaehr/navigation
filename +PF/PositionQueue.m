classdef PositionQueue < handle
    %POSITIONQU Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        queue = zeros(4,4);
    end
    
    methods
        function obj = PositionQueue()
        end
        
        function add(obj, pos, time)
            array = obj.queue(:,2:end);
            obj.queue = horzcat(array, [time pos]');
        end
    end
    
end

