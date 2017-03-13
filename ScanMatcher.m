classdef ScanMatcher
    %SCANMATCHER Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        submapnx = 4;
        submapny = 4;
        initialSearchStep = 5*[0.1 0.1 deg2rad(3)]';
        maxIterations = 40;
        sig1 = -1;
    end
    
    methods
        
        function obj = ScanMatcher(iS, nx, ny)
            obj.initialSearchStep = iS;
            obj.submapnx = min(3,nx);
            obj.submapny = min(3,ny);
        end
        
        %% SCAN Matcher
        % macht den Scan gegen die Karte
        function bestPose =  matchScan(obj, xinit,PCL, map)
            
            
            
            if ~map.hasData
                bestPose = xinit;
                return;
            end
                
            
            bestPose = xinit;
            bestScore = obj.generateScore(xinit, PCL, map);
            searchStep = obj.initialSearchStep;
            iterations = 0;
            
            while iterations < obj.maxIterations
                maxMoveScore = bestScore;
                bestMovePose = bestPose;
                
                P = obj.generateMovesFromPose(bestPose, searchStep);
                for i=1:size(P, 1)
                    testPose = P(i,:);
                    score = obj.generateScore(testPose, PCL, map);
                    if maxMoveScore < score
                        maxMoveScore = score;
                        bestMovePose = testPose;
                    end
                end
                
                if bestScore < maxMoveScore
                    bestScore  = maxMoveScore;
                    bestPose = bestMovePose;
                else
                    searchStep = 0.66*searchStep;
                    iterations = iterations + 1;
                end
                
                fprintf('%i\t%i\n', iterations, bestScore);
                disp(searchStep);
            end
            
            
        end
        
        function P=  generateMovesFromPose(obj, x, s)
                P = repmat(x, 6, 1);
                P(1,1) = P(1,1) - s(1); %backward
                P(2,1) = P(2,1) + s(1); %forward
                P(3,2) = P(3,2) - s(2); %left
                P(4,2) = P(4,2) + s(2); %right
                P(5,3) = P(5,3) - s(3); %rotate left
                P(6,3) = P(6,3) + s(3); %rotate right
        end
        
        function score = generateScore(obj,x, PCL, map)
            
            T1 = rot2(x(3))*PCL';
            T2 = bsxfun(@plus, T1, [x(1) x(2)]');
            T3 = T2';
            
            XPOS = T3(:,1);
            YPOS = T3(:,2);
            
            [XID, YID] = map.getID(XPOS, YPOS);
            
            score = 0;
            S = nan(length(XID), 1);
            parfor i=1:length(XID)
                s = [XPOS(i) YPOS(i)]';
                SM = nan(4,(obj.submapnx+1)*(obj.submapny+1));
                SM = map.getSupMap(XID(i), YID(i), obj.submapnx, obj.submapny, SM);
                try

                    P = SM(3,:) ./ SM(4,:);
                    [v, idx] = max(P);
                    idx = find(P>0.5);
                    if isnan(v)
                        continue;
                    end
                    xmax = SM(1,idx);
                    ymax = SM(2,idx);
                    d = Inf;
                    for j=1:length(xmax)
                        d = min(d, norm(s - [xmax(j) ymax(j)]'));
                    end
%                     d = norm(s - [xmax ymax]');
                    pe = exp(-(d^2));
                    S(i) = pe;
                    %score = score + pe;
                    
                catch ME
                    disp(ME);
                end

            end            
            score =nanmean(S);
            
        end
        
    end
    
end

