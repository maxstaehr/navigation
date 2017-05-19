classdef PF < handle
    %PF Partikelfilter bestimmen des aktuellen Zustands
    %   Der Zustand wird wie folgt beschrieben
    %   [x y theta vx vrot accx accrot]
    
    properties
        nParticels = 100; %Anzahl von Partikeln
        EST; %aktueller Estimate
        EST_1; %Estimate aus dem letzten Durchlauf
        iNG; %selected States aus dem letzen Durchlauf
        PRED;%aktueller Prädiktionsschritt
        
        CS; 
        
        dt = 0.01;
        sigmaWheel = [0.01 0 0]'; %delta Sigma 
        %%
        %Fehler IMU
        sigmaAccX = 0.01;
        sigmaOmega = 0.001;
        %%
        %Fehler Odometrie
        sigmaWheelOdo = 0.01;
        
        sigmaOFTrans = 0.01;
        sigmaOFRot = 0.01;
        
        %%
        %Fehler SLAM
        sigmaSLAMPosX = 0.01;
        sigmaSLAMPosY = 0.01;
        sigmaSLAMPosTheta = deg2rad(1);
        
        sigmaMarkerPosX = 0.01;
        sigmaMarkerPosY = 0.01;
        sigmaMarkerPosTheta = deg2rad(1);
        
        %% 
        %Berechnet sich aus SigmaWheel
        sigmaRot;
        sigmaTrans;
        
        width = 0.525;
        %% 
        %Null Gewicht für interp1 streng monoton steigend
        w0 = 1e-7;
        wRot = 1;
        
        %Parameter Odometrie
        wODO = 1;        
        Pw_odo_trans;
        Pw_odo_rot;
        
        %Parameter Optical Flow
        wOF = 1;        
        Pw_of_trans;
        Pw_of_rot;
                
        %Parameter Imu
        Pw_imu_accx;
        Pw_imu_rot;
        wIMU = 1;
        
        %Parameter Slam
        Pw_slam_x;
        Pw_slam_y;
        Pw_slam_theta;
        wSLAM = 1;        
        
        %Parameter Marker
        Pw_marker_x;
        Pw_marker_y;
        Pw_marker_theta;
        wMARKER = 10;               
        
        % letzte IMU Zeit für Berechung Integral bei 
        % nicht konstanter Aufrufzeit        
        lastTime;
        
        %letzter ODO/IMU Werte für Durchschnittswert aus 
        % letztem Intervall
        vtransc1;
        vrotc1;
        vtrans1;
        vrot1;
        accx1; 
        omega1;

        %resultulierende Gewichtung
        weight;
                
    end
    
    methods
        
        function obj = PF(startTime, pos)
            obj.EST = zeros(7,obj.nParticels);
            obj.EST_1 = zeros(7,obj.nParticels);
            obj.iNG = ones(1, obj.nParticels);
            obj.PRED = zeros(7,obj.nParticels);
            obj.CS = zeros(7,1);
            
            obj.Pw_odo_trans = zeros(1,obj.nParticels);
            obj.Pw_odo_rot = zeros(1,obj.nParticels);
    
            obj.Pw_imu_accx = zeros(1,obj.nParticels);
            obj.Pw_imu_rot = zeros(1,obj.nParticels);
            
            obj.weight = zeros(1,obj.nParticels);
            obj.lastTime = startTime;
            
            %% Berechnung Standardabweichung Rotation und Translatorik
            % DESCRIPTIVE TEXT
            obj.sigmaRot = 2*obj.sigmaWheelOdo/obj.width;
            obj.sigmaTrans = obj.sigmaWheelOdo;
            obj.resetMarkerMeasurement(pos(1), pos(2), pos(3));
 
        end
        function new = copy(this)
            % Instantiate new object of the same class.
            new = PF(0, [0 0 0]);%feval(class(this));

             % Copy all non-hidden properties.
            p = properties(this);
            for i = 1:length(p)
            new.(p{i}) = this.(p{i});
            end
        end
        
        function zeroWeights(obj)            
            obj.Pw_odo_trans = zeros(1,obj.nParticels);
            obj.Pw_odo_rot = zeros(1,obj.nParticels);
            
            obj.Pw_of_trans = zeros(1,obj.nParticels);
            obj.Pw_of_rot = zeros(1,obj.nParticels);
    
            obj.Pw_imu_accx = zeros(1,obj.nParticels);
            obj.Pw_imu_rot = zeros(1,obj.nParticels);
            
            obj.Pw_slam_x = zeros(1,obj.nParticels);
            obj.Pw_slam_y = zeros(1,obj.nParticels);
            obj.Pw_slam_theta = zeros(1,obj.nParticels);
        
            obj.Pw_marker_x = zeros(1,obj.nParticels);
            obj.Pw_marker_y = zeros(1,obj.nParticels);
            obj.Pw_marker_theta = zeros(1,obj.nParticels);        
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
            cmdVelRotNoise = (velWheelRechts - velWheelLinks)/obj.width;
            
            P = obj.PRED;
            
            for i=1:obj.nParticels
                
                
                
                trans = cmdVelTransNoise(:,i)';                
                rot = cmdVelRotNoise(:,i)';
                
                %% Berechnung delta Weg
                f = [1/6 1/2 1 0];
                p= fliplr([0 trans]).*f;                        
                dtrans = polyval(p,obj.dt);
                dtrans = max([dtrans 0]);

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
        
        function plotResPos(obj, axres)
            
            axes(axres);
            plot(obj.CS(1),obj.CS(2), 'or'); 
        end
        
        function plotPaticle(obj, ax, axhist, axhist2)
            axes(ax);
            plot(ax, obj.EST_1(1,:), obj.EST_1(2,:), 'xr');
            plot(ax, obj.PRED(1,:), obj.PRED(2,:), 'og');
            plot(ax, obj.EST(1,:), obj.EST(2,:), '*m');
            
            for i=1:obj.nParticels
                line([obj.EST_1(1,i) obj.PRED(1,i)], ...
                    [obj.EST_1(2,i) obj.PRED(2,i)], ...
                    'Color', [1.0 0 0]);
                
%                 line([obj.EST(1, obj.iNG(i)) obj.PRED(1,i)], ...
%                     [obj.EST(2, obj.iNG(i)) obj.PRED(2,i)], ...
%                     'Color', [0 1.0 0]);
            end
            
%             
%             legend('EST_1', 'PRED', 'EST');
            

            
            axes(axhist);
            cla(axhist);
            var1 = var(obj.EST_1(5,:));
            var3 = var(obj.PRED(5,:));
            var2 = var(obj.EST(5,:));
            Y = [var1 var3 var2];
            bar(Y);
%             legend('EST_1', 'PRED', 'EST');
            
            axes(axhist2);
            cla(axhist2);
            var4 = var(obj.EST_1(2,:));
            var5 = var(obj.PRED(2,:));
            var6 = var(obj.EST(2,:));       
            Y = [var4 var5 var6;];
            bar(Y);
%             legend('EST_1', 'PRED', 'EST');
%             fprintf('EST1 %f\tEST %f\tPRED %f\n', var1, var2, var3);
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
       
            
            %% Berechnung Fehler
            %sigmaRot, sigmaTrans
            varOdoTrans = obj.sigmaTrans^2;
            varOdoRot = obj.sigmaRot^2;
            
            ErrorTrans = (vtrans(1) - obj.PRED(4,:)).^2;
            ErrorRot = (vrot(1) - obj.PRED(5,:)).^2;
            
            P = (1/sqrt(2*pi*varOdoTrans)) * exp(-ErrorTrans ./(2*varOdoTrans));
            obj.Pw_odo_trans = normalizeP(P);

            P = (1/sqrt(2*pi*varOdoRot)) * exp(-ErrorRot ./(2*varOdoRot));
            obj.Pw_odo_rot = normalizeP(P);
            
        end        
        
        function weightOFMeasurement(obj, vtrans, vrot)
       
            
            %% Berechnung Fehler
            %sigmaRot, sigmaTrans
            varOFTrans = obj.sigmaOFTrans^2;
            varOFRot = obj.sigmaOFTrans^2;
            
            ErrorTrans = (vtrans(1) - obj.PRED(4,:)).^2;
            ErrorRot = (vrot(1) - obj.PRED(5,:)).^2;
            
            P = (1/sqrt(2*pi*varOFTrans)) * exp(-ErrorTrans ./(2*varOFTrans));
            obj.Pw_of_trans = normalizeP(P);

            P = (1/sqrt(2*pi*varOFRot)) * exp(-ErrorRot ./(2*varOFRot));
            obj.Pw_of_rot = normalizeP(P);
            
        end        
        
        function weightInertialMeasurement(obj, accx, vrot)
            
            %% Berechnung Fehler
            O = obj.PRED(5,:);
            A = obj.PRED(6,:);
            ErrorRot =  (vrot - O).^2;
            ErrorAccX = (accx -A).^2;
            
            varAccX = obj.sigmaAccX^2;
            varOmega = obj.sigmaOmega^2;
            
            P_accx = (1/sqrt(2*pi*varAccX)) * exp(-ErrorAccX ./(2*varAccX));
            obj.Pw_imu_accx = normalizeP(P_accx);
            
            P_omega = (1/sqrt(2*pi*varOmega)) * exp(-ErrorRot ./(2*varOmega));
            obj.Pw_imu_rot = normalizeP(P_omega);
        end
        
        function weightMarkerMeasurement(obj, xpos, ypos, thetapos)
            
            %% Berechnung Fehler
            X = obj.PRED(1,:);
            Y = obj.PRED(2,:);
            T = obj.PRED(3,:);
            ErrorX = (xpos - X).^2;
            ErrorY = (ypos - Y).^2;
            ErrorT = (thetapos - T).^2;
            
            
            varX = obj.sigmaMarkerPosX^2;
            varY = obj.sigmaMarkerPosY^2;
            varTheta = obj.sigmaMarkerPosTheta^2;

            
            P_pos_x = (1/sqrt(2*pi*varX)) * exp(-ErrorX ./(2*varX));
            obj.Pw_marker_x = normalizeP(P_pos_x);
            
            P_pos_y = (1/sqrt(2*pi*varY)) * exp(-ErrorY ./(2*varY));
            obj.Pw_marker_y = normalizeP(P_pos_y);
            
            P_pos_theta = (1/sqrt(2*pi*varTheta)) * exp(-ErrorT ./(2*varTheta));
            obj.Pw_marker_theta = normalizeP(P_pos_theta);
            
        end
        
        function weightSlamMeasurement(obj, xpos, ypos, thetapos)
            
            %% Berechnung Fehler
            X = obj.PRED(1,:);
            Y = obj.PRED(2,:);
            T = obj.PRED(3,:);
            ErrorX = (xpos - X).^2;
            ErrorY = (ypos - Y).^2;
            ErrorT = (thetapos - T).^2;
            
            
            varX = obj.sigmaSLAMPosX^2;
            varY = obj.sigmaSLAMPosY^2;
            varTheta = obj.sigmaSLAMPosTheta^2;

            
            P_pos_x = (1/sqrt(2*pi*varX)) * exp(-ErrorX ./(2*varX));
            obj.Pw_slam_x = normalizeP(P_pos_x);
            
            P_pos_y = (1/sqrt(2*pi*varY)) * exp(-ErrorY ./(2*varY));
            obj.Pw_slam_y = normalizeP(P_pos_y);
            
            P_pos_theta = (1/sqrt(2*pi*varTheta)) * exp(-ErrorT ./(2*varTheta));
            obj.Pw_slam_theta = normalizeP(P_pos_theta);
            
        end
        
        function resetMarkerMeasurement(obj, xpos, ypos, thetapos)
            

            obj.PRED(1,:) = xpos;
            obj.PRED(2,:) = ypos;
            obj.PRED(3,:) = thetapos;
            
            obj.EST(1,:) = xpos;
            obj.EST(2,:) = ypos;
            obj.EST(3,:) = thetapos;

            
        end        
                
        function select(obj)
            W = obj.weight +obj.w0;
            CDF = cumsum(W)/sum(W);
            iSelect  = rand(obj.nParticels,1);

            % find the particle that corresponds to each y value (just a look up)
            iNextGeneration = interp1(CDF, 1:obj.nParticels, iSelect, 'nearest', 'extrap');
            obj.iNG = iNextGeneration;

            % copy selected particles for next generation..
            obj.EST_1 = obj.EST;
            obj.EST = obj.PRED(:,iNextGeneration);
        end
        
        function determineBestEst(obj)            
            x_est = 0*obj.CS;
            for i = 1 : obj.nParticels
                x_est = x_est + obj.weight (i) *obj.PRED(:,i);
            end
            obj.CS = x_est;           
        end
        
        function fuseWeights(obj)
             obj.weight = ...
                               obj.wODO * obj.Pw_odo_trans + ...
              obj.wRot * obj.wODO * obj.Pw_odo_rot + ...
                               obj.wOF * obj.Pw_of_trans + ...
              obj.wRot * obj.wOF * obj.Pw_of_rot + ...                
              obj.wRot * obj.wIMU * obj.Pw_imu_rot + ...
                              obj.wIMU * obj.Pw_imu_accx + ...
                              obj.wSLAM * obj.Pw_slam_x + ...
                              obj.wSLAM * obj.Pw_slam_y + ...
              obj.wRot * obj.wSLAM * obj.Pw_slam_theta + ...      
                              obj.wMARKER * obj.Pw_marker_x + ...
                              obj.wMARKER * obj.Pw_marker_y + ...
              obj.wRot * obj.wMARKER * obj.Pw_marker_theta;
              obj.weight = obj.weight ./ sum(obj.weight);
        end
                
        function determineDT(obj, imuTime)
            obj.dt = imuTime - obj.lastTime;
            obj.lastTime = imuTime;
        end
        
        function [vtransc2, vrotc2,  vtrans2, vrot2, accx2, omega2] = determineDeltaState(obj, vtransc, vrotc,  vtrans, vrot, accx, omega)
                if ~isempty(obj.transc1)
                    vtransc2 = (obj.vtransc1 + vtransc) / 2;
                    vrotc2 = (obj.vrotc1+vrotc) / 2;
                    vtrans2= (obj.vtrans1+vtrans) / 2;
                    vrot2 = (obj.vrot1+vrot) / 2;
                    accx2 = (obj.accx1+accx) / 2;
                    omega2 = (obj.omega1+omega) / 2;      
                else
                    vtransc2 = vtransc;
                    vrotc2 = vrotc;
                    vtrans2 = vtrans;
                    vrot2 = vrot;
                    accx2 = accx;
                    omega2 = omega;                    
                end
                obj.vtransc1 = vtransc;
                obj.vrotc1 = vrotc;
                obj.vtrans1 = vtrans;
                obj.vrot1 = vrot;
                obj.accx1 = accx;
                obj.omega1 = omega;
        end

        function predictstep(obj,vtransc, vrotc, imuTime)
            obj.determineDT(imuTime);
            obj.predict(vtransc, vrotc);
            obj.determineBestEst();
            obj.zeroWeights();
        end
        
        function stepIMU(obj,vtrans, vrot, accx, omega)
            obj.weightOdometrieMeasurement(vtrans, vrot);
            obj.weightInertialMeasurement(accx, omega);            
            obj.fuseWeights();
            obj.determineBestEst();
            obj.select();
        end
                
        
%%asynchronious functions  single modality
        function stepMarker(obj,vtrans, vrot, accx, omega, pos)
            obj.weightMarkerMeasurement(pos(1), pos(2), pos(3));
            obj.weightOdometrieMeasurement(vtrans, vrot);
            obj.weightInertialMeasurement(accx, omega);            
            obj.fuseWeights();
            obj.determineBestEst();
            obj.select();            
        end
                
        function stepSlam(obj,vtrans, vrot, accx, omega, pos)            
            obj.weightSlamMeasurement(pos(1), pos(2), pos(3));
            obj.weightOdometrieMeasurement(vtrans, vrot);
            obj.weightInertialMeasurement(accx, omega);            
            obj.fuseWeights();
            obj.determineBestEst();
            obj.select();            
        end
        
        function stepOF(obj,vtrans, vrot, accx, omega, vtransof, vrotof)
            obj.weightOFMeasurement(vtransof, vrotof);
            obj.weightOdometrieMeasurement(vtrans, vrot);
            obj.weightInertialMeasurement(accx, omega);            
            obj.fuseWeights();
            obj.determineBestEst();
            obj.select();            
        end        
        
%%asynchronious functions  2 modalities

        function stepMarkerSlam(obj,vtrans, vrot, accx, omega, pos1, pos2)
            obj.weightMarkerMeasurement(pos1(1), pos1(2), pos1(3));
            obj.weightSlamMeasurement(pos2(1), pos2(2), pos2(3));
            obj.weightOdometrieMeasurement(vtrans, vrot);
            obj.weightInertialMeasurement(accx, omega);            
            obj.fuseWeights();
            obj.determineBestEst();
            obj.select();            
        end
        
        function stepMarkerOF(obj,vtrans, vrot, accx, omega, pos,  vtransof, vrotof)
            obj.weightMarkerMeasurement(pos(1), pos(2), pos(3));
            obj.weightOFMeasurement(vtransof, vrotof);
            obj.weightOdometrieMeasurement(vtrans, vrot);
            obj.weightInertialMeasurement(accx, omega);            
            obj.fuseWeights();
            obj.determineBestEst();
            obj.select();            
        end
                
        function stepSlamOF(obj,vtrans, vrot, accx, omega, pos, vtransof, vrotof)         
            obj.weightOFMeasurement(vtransof, vrotof);
            obj.weightSlamMeasurement(pos(1), pos(2), pos(3));
            obj.weightOdometrieMeasurement(vtrans, vrot);
            obj.weightInertialMeasurement(accx, omega);            
            obj.fuseWeights();
            obj.determineBestEst();
            obj.select();            
        end
%%asynchronious functions  3 modalities        
        function stepMarkerSlamOF(obj,vtrans, vrot, accx, omega,pos1, pos2, vtransof, vrotof)
            obj.weightMarkerMeasurement(pos1(1), pos1(2), pos1(3));
            obj.weightSlamMeasurement(pos2(1), pos2(2), pos2(3));
            obj.weightOFMeasurement(vtransof, vrotof);
            obj.weightOdometrieMeasurement(vtrans, vrot);
            obj.weightInertialMeasurement(accx, omega);            
            obj.fuseWeights();
            obj.determineBestEst();
            obj.select();            
        end   
        

    end
    
end

