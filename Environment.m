classdef Environment
    %ENVIRONMENT Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        L;
    end
    
    methods
        function obj = Environment(L)
            obj.L = L;            
        end
        
        function h =  plot(obj, ax)
            axes(ax);            
            for i=1:length(obj.L)
                P = obj.L{i};
                h = line(P(:,1),P(:,2));
            end
        end
        
    end
    
end

