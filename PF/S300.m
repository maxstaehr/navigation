classdef S300
    %S300 Summary of this class goes here
    %   Detailed explanation goes here
    
    properties        
        A;
        D;
        sigma =0.05;
    end
    
    methods
        function obj = S300(res, openangle)
            N = openangle/res+1;
            obj.A = linspace(0,openangle, N);            
            obj.D = zeros(1, N);           
        end
        
        function plot(obj, ax, x)
            axes(ax);
            
            At = obj.A+x(3);
            X = cos(At) + x(1);
            Y = sin(At) + x(2);            
            for i=1:length(At)
                line([x(1) X(i)],[x(2) Y(i)])
            end
            
            X = obj.D .* cos(At) + x(1);
            Y = obj.D .* sin(At) + x(2);            
            plot(X, Y, '*r', 'MarkerSize', 10)                
            
        end
        
        function plot_local(obj, ax)
            axes(ax);
            if ~isempty(obj.DIR)                
                for i=1:size(obj.DIR, 2)
                    line([obj.O_local(1) obj.O_local(1)+obj.DIR_local(1,i)],[obj.O_local(2) obj.O_local(2)+obj.DIR_local(2,i)])
                end
            end
            if ~isempty(obj.PCL_local)   
                plot(obj.PCL_local(1,:), obj.PCL_local(2,:), '*r', 'MarkerSize', 10)                
            end
        end
        
        

        
        
        
        function obj = raytrace(obj, env, x)   
            
            At = obj.A+x(3);
            o = x(1:2)';
            Dl = 100*[cos(At);
                          sin(At)];
            Da = zeros(1, length(At));
            envL = env.L;
            parfor v = 1:length(At)
                d = o + Dl(:,v);           
                X  = [];
                for oi=1:length(envL)
                    L = envL{oi};
                    [x, y] = polyxpoly([o(1); d(1)], [o(2); d(2)], L(:,1), L(:,2), 'unique');         
                    Xt = [x y];
                    Xt(~any(isnan(Xt),2),:);
                    X = vertcat(X, Xt);
                end
                
                X1 = bsxfun(@minus, X, o');
                N = zeros(size(X1, 1), 1);
                for i=1:length(N)
                    N(i) = norm(X1(i,:));
                end
                [vn, idx] = min(N);
%                 obj.D(v) = vn;

                Da(v) = vn;
            end                         
            N = obj.sigma * randn(size(obj.D, 1), size(obj.D, 2));
            Da = Da + N;
            obj.D = Da;
        end

        
    end
    
end

