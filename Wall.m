classdef Wall
    %WALL Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        V;
        F;
    end
    
    methods
        function obj = Wall(p1, p2)
            obj.V = [p1(1) p1(2) -1;
                        p1(1) p1(2) 1;
                        p2(1) p2(2) -1;
                        p2(1) p2(2) 1];
                    
             obj.F = [1 2 3;
                          2 3 4];
                
        end
        
        function plot(obj, ax)
            axes(ax);
            trisurf(obj.F, obj.V(:,1), obj.V(:,2), obj.V(:,3));
        end
    end
    
end

