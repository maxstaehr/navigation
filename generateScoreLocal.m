function score = generateScoreLocal(obj,x, PCL, map)
            
            T1 = rot2(x(3))*PCL';
            T2 = bsxfun(@plus, T1, [x(1) x(2)]');
            T3 = T2';
            
            XPOS = T3(:,1);
            YPOS = T3(:,2);
            
            [XID, YID] = map.getID(XPOS, YPOS);
                        
            S = nan(length(XID), 1);
            M = [];
            
            submapnxl = obj.submapnx;
            submapnyl = obj.submapny;
            for i=1:length(XID)
                
                SM = nan(4,(submapnxl+1)*(submapnyl+1));
                SM = map.getSupMap(XID(i), YID(i), submapnxl, submapnyl, SM);
                try

                    P = SM(3,:) ./ SM(4,:);
                    [v, idx] = max(P);
                    idx = find(P>0);
                    if isnan(v)
                        continue;
                    end
                    xmax = SM(1,idx);
                    ymax = SM(2,idx);
                    d = Inf;
                    maxIndex = 1;
                    for j=1:length(xmax)
                        s = [XPOS(i) YPOS(i)]';
                        dist = norm(s - [xmax(j) ymax(j)]');
                        if dist < d
                            d = norm(s - [xmax(j) ymax(j)]');
                            maxIndex = j;
                        end
                        
                    end
%                     M = vertcat(M, [XPOS(i) YPOS(i) xmax(maxIndex) ymax(maxIndex)]);
%                     d = norm(s - [xmax ymax]');
                    pe = exp(-(d^2));
                    S(i) = pe;
                    %score = score + pe;
                    
                catch ME
                    disp(ME);
                end

            end           
            
%             axes(obj.ax); grid on; hold on; 
%             map.plotWeightedVoxel(obj.ax);
% %             map.plotMap(obj.ax);
%             plot(T3(:,1), T3(:,2), 'rx');
%             for i=1:size(M,1)
%                 line([M(i,1) M(i,3)], [M(i,2),  M(i,4)]);
%             end
            
            score =nanmean(S);
            
        end