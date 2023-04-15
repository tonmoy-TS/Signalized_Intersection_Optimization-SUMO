%% ECO-AND main simulation script
%   Runs one simulation for a given vph / range / eco_thm configuration.
%   Call directly for a single run, or via main_SUMO_simloop.m for parameter sweeps.
%
%   High-level flow:
%     1. Start SUMO via TraCI (SUMO_Init).
%     2. Pre-compute full TL timing table (globalTLsInformation).
%     3. Each simulation step:
%          a. Classify vehicles: in-range AVs, out-of-range AVs, HDVs.
%          b. For each AV newly entering the detection zone, read its TL state
%             and solve the Optimal Control Problem (OC_SUMO) once — this
%             produces a complete velocity profile for its entire approach.
%          c. Replay that pre-computed profile by commanding setSpeed each step.
%     4. After the 1-hour sim: parse SUMO XML outputs and save results.
%
%   Dependencies:
%     - SUMO with TraCI (sumo-gui must be on the system PATH)
%     - traci4matlab  <https://github.com/pipeacosta/traci4matlab>
%     - eco-and/ subfolder (OCP solver, not distributed — see SUMO_Init.m)
%
%   Global parameters (set here or by main_SUMO_simloop.m before calling):
%     vph      - vehicles per hour per lane  (e.g. 200 / 400 / 600 / 800)
%     range    - DSRC detection range in metres  (e.g. 150 / 300)
%     eco_thm  - optimisation objective: 'min-energy' | 'min-time'
%
%   Author: Tonmoy Sarker

clear all; close all; clc; format compact

%% Start timer
tic

%% Global variables
import traci.constants
global outputfoldername verbose
global envCons mpr vph range rho eco_thm

%% Simulation parameters
% Override these here for a single run; main_SUMO_simloop.m sets them for sweeps.
% mpr = 80;             % Market Penetration Rate (%) — uncomment when using MPR-specific route files
vph     = 600;          % vehicles per hour per lane
range   = 300;          % DSRC detection range (metres)
eco_thm = 'min-energy'; % 'min-energy' | 'min-time'

verbose = 0;         % set to 1 to enable verbose fprintf output in OCP solvers

%% Output folder
% Each run gets its own timestamped subfolder inside SUMO_results/.
% The folder name is also stored in the global outputfoldername so SUMO_Init
% can build the --output-prefix path passed to SUMO at startup.
outputfilepath = fullfile(fileparts(mfilename('fullpath')), 'SUMO_results', filesep);
timestamp = datestr(now, 'yyyy-mm-dd-HH-MM-SS');
% Use these variants instead when mpr is set (MPR-specific route files):
% output_folder = [outputfilepath,timestamp,'-vph_',num2str(vph),'-range_',num2str(range),'-MPR_',num2str(mpr),'-results'];
output_folder = [outputfilepath,timestamp,'-vph_',num2str(vph),'-range_',num2str(range),'-results'];
mkdir(output_folder)
% outputfoldername = [timestamp,'-vph_',num2str(vph),'-range_',num2str(range),'-MPR_',num2str(mpr),'-results\'];
outputfoldername = [timestamp,'-vph_',num2str(vph),'-range_',num2str(range),'-results\'];

%% Initialize SUMO
% SUMO_Init starts sumo-gui via TraCI, wires up the output/detector/route
% file paths, loads physical constants (Env_Const), and subscribes to
% vehicle context around the intersection.
envCons = SUMO_Init();
stepSize = envCons.stepSize;   % simulation step length (s)
numSteps = envCons.numSteps;   % total steps (3600 = 1 hour at 1 s/step)

%% Initialize variables
ii = 1;  % main simulation step counter

% These track which vehicles appeared in the previous step so we can
% detect newly arrived vehicles by set-difference each step.
prevVehIDs_inNtwk  = {};
newVehIDs_inNtwk   = {};
prevVehIDs_inRange = {};
newVehIDs_inRange  = {};

% ecoControlledDetectors: used with ARM-based MPR (see commented block below)
ecoControlledDetectors = {};

% uniqueIDs maps each vehicle's string ID to an integer column index in OCvel.
% MATLAB cell arrays require integer indices; vehicle IDs are strings, so we
% maintain this Map to translate between them.
% uniqueIDs = containers.Map('KeyType','char','ValueType','int8');  % alternative with explicit key/value types
uniqueIDs    = containers.Map();
uniqueIDsAll = containers.Map();  % used when tracking all vehicles in network (see commented block in loop)

% OCvel is a 2D cell array: OCvel{step, vehicleIndex} = speed (m/s).
% The full velocity profile is computed once per vehicle at zone entry and
% stored here; each subsequent step just reads the next cell.
OCvel = {};

% All-network speed/distance logging — uncomment the tracking block in the loop to populate these
velAll  = {};
distAll = {};

% Running counts of how many unique vehicles have entered each tracking scope.
% These are used as column-index offsets when assigning into OCvel/velAll.
numNewVehInNtwk  = 0;
numNewVehInRange = 0;

% tg_rest / tr_rest: remaining green/red at zone entry — used in non-compliant
% vehicle logging (see troubleshooting block below).
tg = 0; tr = 0; ty = 0;
tg_rest = 0; tr_rest = 0;

% Non-compliant vehicle counter — used with nonECOvehIds table (see troubleshooting block below)
nonECOvehCounts = 0;

%% Non-compliant vehicles (troubleshooting)
% When the OCP cannot find a valid trajectory (e.g. vehicle too close to TL,
% infeasible timing window), OCvel will be very short (≤2 entries). Uncomment
% this block and the matching append lines inside the loop to log those cases.
% testVehicle = 'fEW.AV.80.6';
% nc_vehicleIDs= {'fEW.3', 'fWS.2', 'fWN.4', 'fWN.5', 'fNW.9', 'fNW.11','fEW.AV.70.6','fEW.AV.80.6','fSN.AV.90.8'};
% parameters  = {'step',  'ID',    'v0',  'tg',    'tr',    'sgr',  't0',    'tg(rest)','tr(rest)','tlsDistance','acSUMO','acOCP'};
% dataType    = {'uint32','string','cell','double','double','uint8','double','double',  'double',  'double',     'cell',  'cell'};
% nonECOvehIds = table('Size', [0, numel(parameters)], 'VariableTypes', dataType, 'VariableNames', parameters);

%% Pre-compute traffic light timing table
% globalTLsInformation queries SUMO once and returns a lookup table:
%   tlAllSameStateDur{phase+1, tlsIndex+1} = [tg, tr]
% where tg/tr are the full green/red durations for that TL link index at
% that phase. Queried once here rather than every step to avoid TraCI overhead.
% (+1 offsets are needed because SUMO phase/index numbering starts at 0.)
trafficlights = traci.trafficlights.getIDList();
tlsID = trafficlights{1};  % single intersection — only one TL controller
[tlAllRYGStates, tlAllPhaseDurations, tlAllSameStateDur, tlCycleDur] = globalTLsInformation(tlsID);
% [tlAllIndexStates,tlAllPhaseDurations, tlAllSameStateDur] = globalTLsInformation_Y(tlsID); % alternative: with yellow phase

%% Set ego-vehicle speed factors
% traci.vehicle.setSpeedMode('v_0',31);
% traci.vehicle.setSpeedFactor('v_0', 1.0);
% traci.vehicletype.setSpeedDeviation('ev-ego', 0);

%% Configure AV vehicle types
% Disable speed randomization and driver imperfection for all non-HDV types.
% This ensures AVs follow the commanded speed profile exactly; any stochastic
% deviation would break the pre-computed OCP trajectory.
vehicletypes = traci.vehicletype.getIDList();
for i=1:length(vehicletypes)
    if ~contains(vehicletypes{i},'HDV')  % skip Human Driven Vehicles
        traci.vehicletype.setSpeedFactor(vehicletypes{i}, 1.0);     % no speed scaling
        traci.vehicletype.setSpeedDeviation(vehicletypes{i}, 0);    % no random deviation
        traci.vehicletype.setImperfection(vehicletypes{i}, 0);      % no Wiedemann imperfection
        % traci.vehicletype.setMaxSpeed(vehicletypes{i},16);
        % typeMaxSpeed = traci.vehicletype.getMaxSpeed(vehicletypes{i});
    end
end

%% Get E3 (Multi-Entry-Multi-Exit) detector IDs
% E3 detectors define the control zone boundary — vehicles inside them are
% considered 'in range' and get OCP-controlled speed commands.
% SUMO E1 (induction loop) and E2 (lane area) detectors are unused here.
% routes = traci.route.getIDList();
% indloopDetectors = traci.inductionloop.getIDList();  % E1 detector
% lnareaDetectors  = traci.lanearea.getIDList();       % E2 detector
memeDetectors = traci.multientryexit.getIDList();      % E3 detector

%% Set ARM-based MPR (optional)
% Controls how many intersection arms (approach roads) have ECO-AND active.
% mpr_arm = 1 means all 4 arms; 0.25 means 1 arm, etc.
% mpr_arm = 1; % values possible: 0, 0.25, 0.5, 0.75 and 1
% numControlledEdges = 4*mpr_arm;
% if numControlledEdges > 0
%     for i=1:numControlledEdges
%         ecoControlledDetectors{i} = memeDetectors{i};
%     end
% else
%     ecoControlledDetectors ={};
% end

%% =========================================================================
%% Main simulation loop
%% =========================================================================
while ii < numSteps

    traci.simulationStep();  % advance SUMO by one step

    %% Classify vehicles this step
    % Returns: all vehicles in network, AVs inside the E3 zone, AVs outside
    % the zone, all AVs, all HDVs. HDVs are never controlled.
    [allVehIDs, vehIDs_inRange, vehIDs_outRange, vehIDs_AV, vehIDs_HDV] = vehInsideMEMEDetectors(memeDetectors);

    % Out-of-range AVs: command free-flow speed (22 m/s ≈ 79 km/h).
    % Using setSpeed(-1) here causes SUMO to revert to car-following, which
    % produces erratic headway behaviour — a fixed speed is more stable.
    for i = 1:length(vehIDs_outRange)
        traci.vehicle.setSpeed(vehIDs_outRange{i}, 22);           % free-flow cruise
        traci.vehicle.setColor(vehIDs_outRange{i}, [255 255 255 255]);  % white = uncontrolled
    end

    % Detect vehicles that just entered the network or the control zone this step.
    newVehIDs_inNtwk  = allVehIDs(~ismember(allVehIDs, prevVehIDs_inNtwk));
    prevVehIDs_inNtwk = allVehIDs;

    newVehIDs_inRange  = vehIDs_inRange(~ismember(vehIDs_inRange, prevVehIDs_inRange));
    prevVehIDs_inRange = vehIDs_inRange;

    % Randomize initial velocities (optional — disabled)
%     for i= 1:length(newVehIDs_inNtwk)
%         v0 = (envCons.V_max-5) + rand*(5);
%         traci.vehicle.setSpeed(newVehIDs_inNtwk{i},v0);
%     end
% newVehIDs_inRange ={};

    %% Solve OCP for vehicles newly entering the detection zone
    % The OCP is solved once per vehicle at the moment it crosses into the
    % E3 zone. It returns a full speed-vs-time profile for the entire approach.
    if ~isempty(newVehIDs_inRange)
        for i = 1:length(newVehIDs_inRange)

            % Register this vehicle with a unique integer column index in OCvel.
            uniqueIDs(newVehIDs_inRange{i}) = i + numNewVehInRange;

            v_step = traci.vehicle.getSpeed(newVehIDs_inRange{i});  % current speed (m/s) = v0
            traci.vehicle.setColor(newVehIDs_inRange{i}, [0 255 0 255]);  % green = in control zone

            % Get the next traffic signal on this vehicle's route.
            % getNextTLS returns a list; we only care about the nearest one.
            Next_TLS = traci.vehicle.getNextTLS(newVehIDs_inRange{i});
            [tlsID, tlsIndex, tlsDistance, tlsState] = Next_TLS{1}{:};

            % Time remaining in the current TL phase (seconds until next switch).
            tlsTimeToEnd = traci.trafficlights.getNextSwitch(tlsID) - traci.simulation.getTime();
            tlsPhase     = traci.trafficlights.getPhase(tlsID);

            % Look up full green/red cycle durations for this TL link index.
            % +1 on both indices because SUMO phases and link indices start at 0.
            tg = tlAllSameStateDur{tlsPhase+1, tlsIndex+1}(1);  % full green duration (s)
            tr = tlAllSameStateDur{tlsPhase+1, tlsIndex+1}(2);  % full red duration (s)
            %ty = tlAllSameStateDur{tlsPhase+1,tlsIndex+1}(3);  % yellow — unused

            % t0: elapsed time within the current phase (time already spent in this phase).
            % = total phase duration − time remaining until switch.
            tlElapsedTime = tlAllPhaseDurations{tlsPhase+1, tlsIndex+1} - tlsTimeToEnd;

            % sgr flag: 1 = currently green, 0 = currently red.
            % Note: comparing against 'r' directly because the condition
            %   tlsState == 'G' || tlsState == 'g'
            % always evaluates true in MATLAB (non-empty char comparison quirk).
            if tlsState == 'r'
                sgr    = 0;
                tr_rest = tr - tlElapsedTime;  % remaining red time
                tg_rest = tg;                  % full next green
            else
                sgr    = 1;
                tg_rest = tg + ty - tlElapsedTime;  % remaining green time
                tr_rest = tr;                        % full next red
            end

%             if contains(['fSN.AV1.100.1'],newVehIDs_inRange{i})
%                 nonECOvehIds = [nonECOvehIds; {ii,newVehIDs_inRange{i},[],tg,tr,sgr,tlElapsedTime,tg_rest,tr_rest,tlsDistance,[], []}];
%             end

            % Solve the Optimal Control Problem for this vehicle.
            % OC_SUMO dispatches to ECO_AND_min_energy / ECO_AND_min_time based
            % on eco_thm, and returns a full velocity array (tempVel) covering
            % every future step from now until the vehicle passes the stop line.
            [tempVel, tempAcc, fgr] = OC_SUMO(sgr, tlsDistance, v_step, tg, tr, tlElapsedTime);

            % Store the velocity profile in OCvel, aligned to the current step.
            % Rows = simulation steps; columns = per-vehicle index (uniqueIDs).
            OCvel(ii:ii+length(tempVel)-1, i+numNewVehInRange) = num2cell(tempVel);
            % OCacc(ii:ii+length(tempVel)-1,i+numNewVehInRange) = num2cell(tempAcc);

%             if contains(['fEW.AV.80.6','fEW.AV.80.6','fSN.AV.90.8'],newVehIDs_inRange{i})
%                nonECOvehIds = [nonECOvehIds; {ii,newVehIDs_inRange{i},tempVel,tg,tr,sgr,tlElapsedTime,tg_rest,tr_rest,tlsDistance,[], []}];
%             end

            % Flag non-compliant vehicles: if OCP returned ≤2 speed values and
            % the vehicle needed active control (fgr=0), the solver could not
            % find a feasible trajectory. Mark blue for visual identification.
            if length(tempVel) <= 2 && fgr == 0
                traci.vehicle.setColor(newVehIDs_inRange{i}, [0 0 255 255]);  % blue = OCP failed
                %acSUMO = traci.vehicle.getAcceleration(newVehIDs_inRange{i});
                %nonECOvehIds = [nonECOvehIds; {ii,newVehIDs_inRange{i},tempVel,tg,tr,sgr,tlElapsedTime,tg_rest,tr_rest,tlsDistance,[], []}];
            end
        end
    end

    %% Replay pre-computed velocity profiles
    % Each in-range AV receives the speed value stored for this exact step.
    % setSpeed() overrides SUMO's car-following model for this vehicle.
    % If the cell is empty (vehicle is past its computed horizon), fall back
    % to setSpeed(-1) which re-enables SUMO's default car-following logic.
    if ~isempty(vehIDs_inRange) && ii <= length(OCvel)
        for i = 1:length(vehIDs_inRange)
            index = uniqueIDs(vehIDs_inRange{i});

            if ~isempty(OCvel{ii, index})
                traci.vehicle.setSpeed(vehIDs_inRange{i}, OCvel{ii, index});
            else
                traci.vehicle.setSpeed(vehIDs_inRange{i}, -1);  % hand back to SUMO car-following
            end
        end
    end

    %% TO DO: Control vehicles out of range (Downstream flow control)
%     if ~isempty(vehIDs_outRange)
%         for i= 1 : length(vehIDs_outRange)
%             traci.vehicle.setSpeed(vehIDs_outRange{i},-1);
%         end
%     end

    %% Optional: log speed & distance for all vehicles in the network
    % Uncomment to populate velAll / distAll for full trajectory analysis.
%     if ~isempty(newVehIDs_inNtwk)
%         for i=1:length(newVehIDs_inNtwk)
%             uniqueIDsAll(newVehIDs_inNtwk{i}) = i+numNewVehInNtwk;
%         end
%     end
%
%     if ~isempty(allVehIDs)
%         for i= 1 : length(allVehIDs)
%             index = uniqueIDsAll(allVehIDs{i});
%
%             velAll(ii,index) = num2cell(traci.vehicle.getSpeed(allVehIDs{i}));
%             distAll(ii,index) = num2cell(traci.vehicle.getDistance(allVehIDs{i}));
%         end
%     end

    % Advance the cumulative vehicle counter and step index.
    numNewVehInRange = numNewVehInRange + length(newVehIDs_inRange);
    % numNewVehInNtwk =  numNewVehInNtwk + length(newVehIDs_inNtwk);

    steps(ii,1) = ii;
    ii = ii + 1;

end

traci.close();  % flushes all pending XML outputs before SUMO exits (see --quit-on-end in SUMO_Init)

%% Stop the timer
run_time = toc;
disp(['Elapsed time: ' num2str(run_time) ' seconds']);

%% Replace the empty cells with 'NaN' (optional — for post-processing)
% OCvel(cellfun('isempty',OCvel))={nan};
% velAll(cellfun('isempty',velAll))={nan};
% distAll(cellfun('isempty',distAll))={nan};

%% Save results
% readAllData parses the XML outputs written by SUMO during the simulation,
% assembles a table of lane-level metrics (waiting time, fuel, CO2) keyed
% by MPR, generates summary plots, and returns the table for saving.
% output_file=[output_folder,'/',timestamp,'-vph_',num2str(vph),'-range_',num2str(range),'-MPR_',num2str(mpr),'-data.mat'];
% output_file=[output_folder,'/',timestamp,'-vph_',num2str(vph),'-range_',num2str(range),'-data.mat'];
% save(output_file);

finalData = readAllData(output_folder, eco_thm, run_time);
laneData  = [output_folder, '/', timestamp, '-vph_', num2str(vph), '-range_', num2str(range), '-laneDumpdata.mat'];
save(laneData, "finalData");

%% Save non-compliant vehicle information (troubleshooting)
% nonECOvehCounts = height(nonECOvehIds);
% output_Table=[output_folder,'/',timestamp,'-vph_',num2str(vph),'-range_',num2str(range),'-nonECOvehIds.mat'];
% % % writetable(nonECOvehIds,output_Table,'Delimiter','\t','WriteRowNames',true);
% save(output_Table, 'nonECOvehIds', 'nonECOvehCounts', 'tlCycleDur', 'ecoand');

%% Optional: plot vehicle trajectories
% fig = figure;
% plotTLsPhaseDiagram(tlAllRYGStates,tlAllPhaseDurations);
%
% figtitle = [output_folder,'/',timestamp,'-vph_',num2str(vph),'-range_',num2str(range),'-trajectories.png'];
% plot(steps*stepSize,cell2mat(distAll),'b');
% xlabel('Time(s)'); ylabel('Distance (m)');
% saveas(fig,figtitle)

%close all
