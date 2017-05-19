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
        
    end
    
    methods
        function obj = PositionEstimator(nStates, startTime, pos)
            obj.listOfStates = cell(1,nStates);            
            obj.listOfStatePredicts = cell(1,nStates);
            
            obj.listOfSlam = cell(1,nStates);
            obj.listOfMarker = cell(1,nStates);
            obj.listOfOF = cell(1,nStates);
            obj.listOfIMU = cell(1,nStates);
            
            obj.indexRecalculate = nStates;
            
            for i=1:nStates
                pf = PF(startTime, pos);
                obj.listOfStates{i} = pf;
            end
            obj.currentState = pf;
        end
        
        function insertState(obj, state)            
            ll = obj.listOfStates(2:end);
            ll{end+1} = state;
            obj.listOfStates = ll;
        end
               
        function insertStatePredict(obj, state)            
            ll = [obj.listOfStatePredicts(2:end) state];
            obj.listOfStatePredicts = ll;
        end        
        
        function insertStateIMU(obj, state)            
            ll = [obj.listOfIMU(2:end) state];
            obj.listOfIMU = ll;
            
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

        end           
        
        
         function predictstep(obj,vtransc, vrotc, imuTime)             
             obj.insertStatePredict(struct('vtransc', vtransc, 'vrotc', vrotc, 'imuTime', imuTime));
             obj.currentState.predictstep(vtransc, vrotc, imuTime);
         end
            
%%
% step Funktionen
         function stepIMU(obj,vtrans, vrot, accx, omega)
             obj.indexRecalculate = length(obj.listOfStates);
             obj.insertStateIMU(struct('vtrans', vtrans, 'vrot', vrot, 'accx', accx, 'omega', omega));
             obj.currentState.stepIMU(vtrans, vrot, accx, omega);             
             obj.insertState(obj.currentState.copy());             
         end
         
         function stepIMUAsynchron(obj,vtrans, vrot, accx, omega)
             obj.indexRecalculate = length(obj.listOfStates);
             obj.insertStateIMU(struct('vtrans', vtrans, 'vrot', vrot, 'accx', accx, 'omega', omega));                         
         end         
         
         function stepSlam(obj, pos, posTime)    
             %% Einhängen der Daten
             % finde zeitschritt, zu dem die Eingangsdaten passen
             % und hänge die Positionsdaten ein
             % setze minmale Index ab dem erneuert werden muss
             for i=2:length(obj.listOfIMU)
                 if obj.listOfStates{i-1}.lastTime <= posTime && ...
                    obj.listOfStates{i}.lastTime > posTime
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
                 if obj.listOfStates{i-1}.lastTime <= posTime && ...
                    obj.listOfStates{i}.lastTime > posTime
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
                 if obj.listOfStates{i-1}.lastTime <= posTime && ...
                    obj.listOfStates{i}.lastTime > posTime
                    %Zeitunkt gefunden, hänge Daten ein
                    obj.listOfOF{i} = struct('vtrans', vtrans, 'vrot', vrot);
                    obj.indexRecalculate = min([obj.indexRecalculate i]);
                 end                         
             end     
         end         
         
         function recalculate(obj)
             state = copy(obj.listOfStates{obj.indexRecalculate-1});
             for i=obj.indexRecalculate:length(obj.listOfOF)
                 %% Bestimmen der Fälle
                 %Welcher Eingansdaten lagen zu dem Datum in der
                 %Vergangenheit an?
                 pred = obj.listOfStatePredicts{i};
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

