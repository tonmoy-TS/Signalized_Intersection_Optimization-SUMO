function [allVehIDs, vehIDs_inRange, vehIDs_outRange, vehIDs_AV, vehIDs_HDV] = vehInsideMEMEDetectors(memeDetectors)
%VEHINSIDEMEMEDECTORS  Classify all active vehicles by type and zone each step.
%
%   Called every simulation step from main_SUMO.m to partition vehicles into
%   the groups that drive control decisions:
%
%     allVehIDs      - all vehicles currently in the SUMO network
%     vehIDs_inRange - AVs inside the E3 detector zone (OCP-controlled)
%     vehIDs_outRange- AVs outside the E3 zone (set to free-flow speed)
%     vehIDs_AV      - all AVs in the network (in- and out-of-range)
%     vehIDs_HDV     - all HDVs in the network (never controlled)
%
%   AV vs HDV classification is based on vehicle ID naming convention:
%   any ID containing 'HDV' is treated as human-driven; all others are AVs.
%   This mirrors how the route files assign IDs (e.g. 'fEW.HDV.1' vs 'fEW.AV.80.1').
%
%   Input:
%     memeDetectors - cell array of E3 detector IDs (from traci.multientryexit.getIDList)

global envCons
import traci.constants

%% Initialise output lists
vehIDs_inRange  = {};
vehIDs_outRange = {};
vehIDs_HDV      = {};
vehIDs_AV       = {};

% Full network vehicle list — ground truth for all vehicles this step.
allVehIDs = traci.vehicle.getIDList();
% allVehicleTypes = traci.vehicletype.getIDList();

%% Step 1: collect AVs inside the E3 detector zone
% Each E3 detector covers one approach arm (one direction × one lane group).
% Querying all detectors and merging their vehicle lists gives the complete
% set of vehicles within the DSRC range boundary on all arms.
if ~isempty(memeDetectors)

    % Query each detector for the vehicles it detected in the last step.
    for i = 1:length(memeDetectors)
        memeVehIDs{i} = traci.multientryexit.getLastStepVehicleIDs(memeDetectors{i});
    end

    % Flatten across detectors, keeping only AVs.
    % HDVs are excluded here because we never command their speed; including
    % them in vehIDs_inRange would cause spurious setSpeed calls in main_SUMO.m.
    % Vehicle type is inferred from the ID string rather than a getTypeID call
    % to avoid a per-vehicle TraCI round-trip each step.
    for i = 1:numel(memeVehIDs)
        for j = 1:numel(memeVehIDs{i})
            %vehType = traci.vehicle.getTypeID(memeVehIDs{i}{j});  % alternative: explicit type query
            if ~contains(memeVehIDs{i}{j}, 'HDV')
                vehIDs_inRange{end+1} = memeVehIDs{i}{j};
            end
        end
    end
end

%% Step 2: split the full network list into AVs and HDVs
for i = 1:numel(allVehIDs)
    if contains(allVehIDs{i}, 'HDV')
        vehIDs_HDV{end+1} = allVehIDs{i};
    else
        vehIDs_AV{end+1} = allVehIDs{i};
    end
end

%% Step 3: out-of-range AVs = all AVs minus those already inside the E3 zone
vehIDs_outRange = setdiff(vehIDs_AV, vehIDs_inRange);

end
