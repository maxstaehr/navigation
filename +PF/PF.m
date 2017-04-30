classdef PF < handle
    %PF Partikelfilter bestimmen des aktuellen Zustands
    %   Der Zustand wird wie folgt beschrieben
    %   [x y theta vx vrot accx accrot]
    
    properties
        nParticels = 1e2; %Anzahl von Partikeln
        EST; %aktueller Estimate
        PRED;%aktueller Prädiktionsschritt
        dt = 0.01;
        sigmaWheel = [0.1 0 0]'; %delta Sigma pro Sekunde
        width = 1;
        
        Pw_odo;
        sigmaWheelOdo = 0.01;
        
        weight;
    end
    
    methods
        function obj = PF()
            obj.EST = zeros(7,obj.nParticels);
            obj.PRED = zeros(7,obj.nParticels);
            
            obj.Pw_odo = zeros(1,obj.nParticels);
            obj.weight = zeros(1,obj.nParticels);
            
        end
        %% Prädiktion
        % Prädiziert den aktuellen Zustand aus dem aktuellen Estimate
        % cmdVelTrans = [vx accx jx]
        % cmdVelRot = [vrot accrot]
        
        function predict(obj, cmdVelTrans, cmdVelRot)
            %% Berechnung von Einzelradgrößen
            % Als erstes muss aus der aktuellen rotatorischen und
            % translatorischen Geschwindigkeit die Radgeschwindigkeit
            % berechnet werden, da diese die Fehlerursache für den
            % Zustandsübergang sind
            cmdVelRotLinks = [-0.5*cmdVelRot*obj.width; 0];
            cmdVelRotRechts = [0.5*cmdVelRot*obj.width; 0];
            cmdVelWheelLinks = cmdVelTrans + cmdVelRotLinks;
            cmdVelWheelRechts = cmdVelTrans + cmdVelRotRechts;
            
            %% Berechnung Noise
            % Nun wird das entsprechende Systemrauschen erzeugt
            Nlinks = bsxfun(@times, obj.sigmaWheel, randn(3, obj.nParticels));
            Nrechts = bsxfun(@times, obj.sigmaWheel, randn(3, obj.nParticels));
            
            velWheelLinks = bsxfun(@plus, cmdVelWheelLinks, Nlinks);
            velWheelRechts = bsxfun(@plus, cmdVelWheelRechts, Nrechts);
            
            cmdVelTransNoise = (velWheelLinks + velWheelRechts)/2;
            cmdVelRotNoise = (velWheelRechts - velWheelLinks)/2;
            
            P = obj.PRED;
            
            for i=1:obj.nParticels
                
                
                
                trans = cmdVelTransNoise(:,i)';
                rot = cmdVelRotNoise(:,i)';
                
                %% Berechnung delta Weg
                f = [1/6 1/2 1 0];
                p= fliplr([0 trans]).*f;                        
                dtrans = polyval(p,obj.dt);

                p = fliplr([0 rot]).*f;                        
                drot = polyval(p,obj.dt);

                %% Berechnung delta Geschwindigkeit
                f = [1/2 1];
                p = fliplr(trans(2:3)).*f;                        
                dvtrans = polyval(p,obj.dt);

                p = fliplr(rot(2:3)).*f;                        
                dvrot = polyval(p,obj.dt);           
                
                %% Berechnung delta Beschleunigung                 
                daccvtrans = trans(3)*obj.dt;                        
                daccvrot = rot(3)*obj.dt;           
                
                
                
                xpos =    obj.EST(1,i);
                ypos =    obj.EST(2,i);
                theta =    obj.EST(3,i);
                
                

                P(1,i) = cos(theta + drot)*dtrans + xpos;
                P(2,i) = sin(theta + drot)*dtrans + ypos;
                P(3,i) = theta + drot;              
                P(4,i) = dvtrans+trans(1);
                P(5,i) = dvrot+rot(1);
                P(6,i) = daccvtrans + trans(2);
                P(7,i) = daccvrot + rot(2);
            end
            obj.PRED = P;
                                    
        end
        
        function plotPaticle(obj, ax)
            plot(ax, obj.EST(1,:), obj.EST(2,:), 'xr');
            plot(ax, obj.PRED(1,:), obj.PRED(2,:), 'og');
            legend('EST', 'PRED');
        end
        function [ PR ] = normalizeP( P )
            %UNTITLED7 Summary of this function goes here
            %   Detailed explanation goes here
            sumP = sum(P);
            if sumP > 0
                PR = P./sumP;    
            else
                PR = P;    
            end
        end
        

        
        function weightOdometrieMeasurement(obj, vtrans, vrot)
            %% Bestimme aktuelle Radgeschwindigkeit
            % Bestimmt die aktuelle Radgeschwindigkeit und vergleicht dann
            % linkes und rechtes Rad gegen die Schätzung
            velRotLinks = -0.5*vrot(1)*obj.width;
            velRotRechts = 0.5*vrot(1)*obj.width;
            velWheelLinks = vtrans(1) + velRotLinks;
            velWheelRechts = vtrans(1) + velRotRechts;
            
           
            velRotLinksEst = -0.5*obj.EST(5,:)*obj.width;
            velRotRechtsEst = 0.5*obj.EST(5,:)*obj.width;
            velWheelLinksEst = obj.EST(4,:) + velRotLinksEst;
            velWheelRechtsEst = obj.EST(4,:) + velRotRechtsEst;
            
            %% Berechnung Fehler
            Error = 0.5*((velWheelLinks - velWheelLinksEst).^2 + ...
                (velWheelRechts -velWheelRechtsEst).^2 );
            
            
            varOdo = obj.sigmaWheelOdo^2;
            P = (1/sqrt(2*pi*varOdo)) * exp(-Error ./(2*varOdo));
            obj.Pw_odo = normalizeP(P);
    
        end
        
        function weightInertialMeasurement(obj, accx, vrot)
        end
        
        function weightMarkerMeasurement(obj, xpos, ypos, thetapos)
        end
        
        function weightSlamMeasurement(obj, xpos, ypos, thetapos)
        end
        
        function weightKinexonMeasuremnt(obj, xpos, ypos, thetapos)
        end
        
        function select(obj)
            CDF = cumsum(obj.weight)/sum(obj.weight);
            iSelect  = rand(obj.nParticels,1);

            % find the particle that corresponds to each y value (just a look up)
            iNextGeneration = interp1(CDF, 1:obj.nParticels, iSelect, 'nearest', 'extrap');

            % copy selected particles for next generation..
            obj.EST = obj.EST(:,iNextGeneration);
        end
        
        function fuseWeights(obj)
             obj.weight = obj.Pw_odo;
        end

        function step(obj, vtrans, vrot)
            obj.predict(vtrans, vrot);
            obj.EST = obj.PRED;
            obj.weightOdometrieMeasurement(vtrans, vrot);
            obj.fuseWeights();
            obj.select();
        end
        
    end
    
end

