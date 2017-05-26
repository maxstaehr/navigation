classdef PositionEstimator < handle
    %POSITIONESTIMATOR Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        listOfStates = {};
        listOfSlam = {};
        listOfOF = {};
        listOfMarker = {};
        listOfStatePredicts = {};
        listOfIMU = {};
        currentState;
        indexRecalculate;
        
        %% CMD Delay
        % Bestimmt den Delay zwischen der Pos
        cmdDelay = 6;
        
    end
    
    methods
        function obj = PositionEstimator(nStates, pos)
            obj.listOfStates = cell(1,nStates);            
            obj.listOfStatePredicts = cell(1,nStates);
            
            obj.listOfSlam = cell(1,nStates);
            obj.listOfMarker = cell(1,nStates);
            obj.listOfOF = cell(1,nStates);
            obj.listOfIMU = cell(1,nStates);
            
            obj.indexRecalculate = nStates;
            

            pf = PF( pos);
            obj.currentState = pf.copy();

        end
        
               
        function insertStatePredict(obj, state)            
             ll = obj.listOfStatePredicts(2:end);
            ll{end+1} = state;
            obj.listOfStatePredicts = ll;
            
             %% verschieben der anderen eingangsdaten
            ll = obj.listOfSlam(2:end);
            ll{end+1} = {};
            obj.listOfSlam = ll;
            
            ll = obj.listOfOF(2:end);
            ll{end+1} = {};
            obj.listOfOF = ll;      
            
            ll = obj.listOfMarker(2:end);
            ll{end+1} = {};
            obj.listOfMarker = ll;
                        
            ll = obj.listOfStates(2:end);
            ll{end+1} = {};
            obj.listOfStates = ll;

        end        
        
        function insertStateIMU(obj, state)            
            ll = [obj.listOfIMU(2:end) state];
            obj.listOfIMU = ll;           
        end           
        
        function [vtrans, vrot] = convertDistance2Velocity(obj, distL, distR, imuTime)            
            lastPredict = obj.listOfIMU{end-1};
            if ~isempty(lastPredict)
                dt = imuTime - lastPredict.imuTime;
                vL =  (distL - lastPredict.distL)/dt;
                vR =  (distR - lastPredict.distR)/dt;            
                vrot = (vR - vL)/obj.currentState.width;
                vtrans = (vR + vL)/2;     
            else
                vrot = 0;
                vtrans = 0;
            end
        end
        
        
         function predictstep(obj,vtransc, vrotc, imuTime)             
             obj.insertStatePredict(struct('vtransc', vtransc, 'vrotc', vrotc,  'imuTime', imuTime));
%              obj.currentState.predictstep(vtransc, vrotc, imuTime);
         end
            
%%
% step Funktionen
         
         function stepIMUVel(obj,vtrans, vrot, accx, omega)
             obj.indexRecalculate = length(obj.listOfStates);
             obj.insertStateIMU(struct('vtrans', vtrans, 'vrot', vrot, 'accx', accx, 'omega', omega));                         
         end         
         
         function stepIMUDist(obj, distL, distR, accx, omega, imuTime)
              [vtrans, vrot] = obj.convertDistance2Velocity( distL, distR, imuTime);
             obj.indexRecalculate = length(obj.listOfStates);             
             obj.insertStateIMU(struct('vtrans', vtrans, 'vrot', vrot, 'accx', accx, 'omega', omega, 'distL', distL, 'distR', distR, 'imuTime', imuTime));                         
         end             
         
         function stepSlam(obj, pos, posTime)    
             %% Einhängen der Daten
             % finde zeitschritt, zu dem die Eingangsdaten passen
             % und hänge die Positionsdaten ein
             % setze minmale Index ab dem erneuert werden muss
             for i=2:length(obj.listOfIMU)
                 if ~isempty(obj.listOfStatePredicts{i-1})  && ...
                     ~isempty(obj.listOfStatePredicts{i}) && ...
                     obj.listOfStatePredicts{i-1}.imuTime < posTime && ...
                    obj.listOfStatePredicts{i}.imuTime >= posTime
                    %Zeitunkt gefunden, hänge Daten ein
                    obj.listOfSlam{i} = struct('pos', pos);
                    obj.indexRecalculate = min([obj.indexRecalculate i]);
                 end                         
             end                                       
         end
         
         function stepMarker(obj, pos, posTime)
             %% Einhängen der Daten
             % finde zeitschritt, zu dem die Eingangsdaten passen
             % und hänge die Positionsdaten ein
             % setze minmale Index ab dem erneuert werden muss
             for i=2:length(obj.listOfIMU)
                 if ~isempty(obj.listOfStatePredicts{i-1})  && ...
                     ~isempty(obj.listOfStatePredicts{i}) && ...
                     obj.listOfStatePredicts{i-1}.imuTime < posTime && ...
                    obj.listOfStatePredicts{i}.imuTime >= posTime
                    %Zeitunkt gefunden, hänge Daten ein
                    obj.listOfMarker{i} = struct('pos', pos);
                    obj.indexRecalculate = min([obj.indexRecalculate i]);
                 end                         
             end     
         end
         
         function stepOF(obj,vtrans, vrot, posTime)
             %% Einhängen der Daten
             % finde zeitschritt, zu dem die Eingangsdaten passen
             % und hänge die Positionsdaten ein
             % setze minmale Index ab dem erneuert werden muss
             for i=2:length(obj.listOfIMU)
                 if ~isempty(obj.listOfStatePredicts{i-1})  && ...
                     ~isempty(obj.listOfStatePredicts{i}) && ...
                     obj.listOfStatePredicts{i-1}.imuTime < posTime && ...
                    obj.listOfStatePredicts{i}.imuTime >= posTime
                    %Zeitunkt gefunden, hänge Daten ein
                    obj.listOfOF{i} = struct('vtrans', vtrans, 'vrot', vrot);
                    obj.indexRecalculate = min([obj.indexRecalculate i]);
                 end                         
             end     
         end         
         
         function recalculate(obj)
             s = obj.listOfStates{obj.indexRecalculate-1};
             if ~isempty(s)
                 state = copy(obj.listOfStates{obj.indexRecalculate-1});
             else
                state = copy(obj.currentState);
             end
             
             for i=obj.indexRecalculate:length(obj.listOfOF)
                 %% Bestimmen der Fälle
                 %Welcher Eingansdaten lagen zu dem Datum in der
                 %Vergangenheit an?
                 % Delay des CMD 
                 %finde nicht leeren Prädiktionsschritt beim Befüllen
                 pred = obj.listOfStatePredicts{i - obj.cmdDelay};
                 if isempty(pred)
                     for j = i - obj.cmdDelay:length(obj.listOfStatePredicts)
                        pred = obj.listOfStatePredicts{i};
                        if ~isempty(pred)
                            break;
                        end
                     end
                 end
                 
                 imu = obj.listOfIMU{i};
                 
                 hasO = ~isempty(obj.listOfOF{i});
                 hasM = ~isempty(obj.listOfMarker{i});
                 hasS = ~isempty(obj.listOfSlam{i});
                 

                 m =  obj.listOfMarker{i};
                 s =  obj.listOfSlam{i};
                 o =  obj.listOfOF{i};
                 
                 state.predictstep(pred.vtransc, pred.vrotc, pred.imuTime);
                 if  hasM && ~hasS && ~hasO                                     
                     state.stepMarker(imu.vtrans, imu.vrot, imu.accx, imu.omega, m.pos);
                 elseif   ~hasM && hasS && ~hasO
                     state.stepSlam(imu.vtrans, imu.vrot, imu.accx, imu.omega, s.pos)  
                 elseif   ~hasM && ~hasS && hasO
                     state.stepOF(imu.vtrans, imu.vrot, imu.accx, imu.omega, o.vtransof, o.vrotof)
                 elseif   hasM && hasS && ~hasO
                     state.stepMarkerSlam(imu.vtrans, imu.vrot, imu.accx, imu.omega, m.pos, s.pos)
                 elseif   hasM && ~hasS && hasO
                     state.stepMarkerOF(imu.vtrans, imu.vrot, imu.accx, imu.omega, m.pos,  o.vtransof, o.vrotof)
                 elseif   ~hasM && hasS && hasO
                     state.stepSlamOF(imu.vtrans, imu.vrot, imu.accx, imu.omega, s.pos, o.vtransof, o.vrotof)  
                 elseif   hasM && hasS && hasO
                     state.stepMarkerSlamOF(imu.vtrans, imu.vrot, imu.accx, imu.omega,m.pos, s.pos, o.vtrans, o.vrot)
                 else
                     state.stepIMU(imu.vtrans, imu.vrot, imu.accx, imu.omega)
                 end
                 
                 %save the state
                 obj.listOfStates{i} = copy(state);
             end
             %den letzten Zustand als aktuellen abspeichern
             obj.currentState = copy(obj.listOfStates{end});
                                       
         end
        
        
        
        
    end
    
end

