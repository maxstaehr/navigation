classdef ScanMatcher
    %SCANMATCHER Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        submapnx = 4;
        submapny = 4;
        initialSearchStep = 5*[0.1 0.1 deg2rad(3)]';
        maxIterations = 16;
        sig1 = -1;
        ax;
    end
    
    methods
        
        function obj = ScanMatcher(iS, nx, ny)
            obj.initialSearchStep = iS;
            obj.submapnx = min(3,nx);
            obj.submapny = min(3,ny);
%             figure;
%             obj.ax = gca();
        end
        
        %% SCAN Matcher
        % macht den Scan gegen die Karte
        function [bestPose, bestScore] =  matchScan(obj, xinit,PCL, map)
            
            
            
            if ~map.hasData
                bestPose = [0 0 0];
                bestScore = 0;
                return;
            end
                
            R = cell(size(xinit,1),1);
            for posIndex=1:size(xinit,1)
            
                bestPose = xinit(posIndex,:);
                bestScore = obj.generateScore(bestPose, PCL, map);
                searchStep = obj.initialSearchStep;
                iterations = 0;

                while iterations < obj.maxIterations
                    maxMoveScore = bestScore;
                    bestMovePose = bestPose;

                    P = obj.generateMovesFromPose(bestPose, searchStep);
                    SCORES = zeros(size(P,1), 1);
                    parfor i=1:size(P, 1)
                        testPose = P(i,:);
                        SCORES(i) = obj.generateScore(testPose, PCL, map);
                    end   
                    
                    for i=1:size(P, 1)            
                         testPose = P(i,:);
                         score = SCORES(i) ;
                        if maxMoveScore < score
                            maxMoveScore = score;
                            bestMovePose = testPose;
                        end
                    end

                    if bestScore < maxMoveScore
                        bestScore  = maxMoveScore;
                        bestPose = bestMovePose;
                    else
                        searchStep = 0.5*searchStep;
                        iterations = iterations + 1;
                    end
                    
                end
                R{posIndex} = {bestPose, bestScore};

            end
            value = R{1};
            bestScore = value{2};
            bestPose = value{1};
            for posIndex=1:size(xinit)
                value = R{posIndex};
                if value{2} > bestScore
                    bestScore = value{2};
                    bestPose = value{1};
                end
            end
            fprintf('%2.5d\n', 100*bestScore);
                    
        end
        
        function P=  generateMovesFromPose(obj, x, s)
                P = repmat(x, 7, 1);
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
                        
            S = nan(length(XID), 1);
            Dist = nan(length(XID), 1);
            M = [];
            
            submapnxl = obj.submapnx;
            submapnyl = obj.submapny;
            for i=1:length(XID)
                
                SM = nan(4,(submapnxl+1)*(submapnyl+1));
                SM = map.getSupMap(XID(i), YID(i), submapnxl, submapnyl, SM);
                s = [XPOS(i) YPOS(i)]';
                try

                    P = SM(3,:) ./ SM(4,:);
                    [v, idx] = max(P);
                    idx = find(P>0.8);
                    if isnan(v) || isempty(idx)
                        continue;
                    end
                    xmax = SM(1,idx);
                    ymax = SM(2,idx);                    
                    dist = norm(s - [xmax(1) ymax(1)]');
                    maxIndex = 1;
                    for j=1:length(xmax)
                        d = dist; 
                        dist = min(dist,norm(s - [xmax(j) ymax(j)]'));                       
                        if dist < d                            
                            maxIndex = j;
                        end
                        
                    end
                    M = vertcat(M, [XPOS(i) YPOS(i) xmax(maxIndex) ymax(maxIndex)]);
%                     d = norm(s - [xmax ymax]');

                    %score = score + pe;
                    Dist(i) = dist;
                catch ME
                    disp(ME);
                end

            end           
            Dist(isnan(Dist(:,1)),:)=[];
            y1 = quantile(Dist,0.9);
%             y2 = quantile(Dist,0.3);
             idx1 = find(Dist<y1);
%             idx2 = find(Dist>y2);
%             idx = intersect(idx1, idx2);
            Dist = Dist(idx1);
            M = M(idx,:);
            
            pe = exp(-(Dist.^2));
            sum = cumsum(pe);
            score = sum(end) / size(PCL,1);
            
%             figure; hold on;
%             grid on; hold on; ax = gca;
%             map.plotWeightedVoxel(ax);
% %             map.plotMap(obj.ax);
%             plot(T3(:,1), T3(:,2), 'rx');
%             for i=1:size(M,1)
%                 line([M(i,1) M(i,3)], [M(i,2),  M(i,4)]);
%             end
            
%             score =mean(pe);
            
        end
        
    end
    
end

