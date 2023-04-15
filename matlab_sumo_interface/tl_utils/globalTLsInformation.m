function [tlAllRYGStates, tlAllPhaseDurations, tlAllSameStateDur, tlCycleDur] = globalTLsInformation(tlsID)
%GLOBALTLSINFORMATION  Pre-compute green/red duration lookup table for all TL phases.
%
%   ACTIVE VERSION — called once at the start of main_SUMO.m.
%   Returns a cached table so that per-vehicle TL timing can be looked up
%   in O(1) during the simulation loop, avoiding repeated TraCI queries.
%
%   Yellow phases are folded into green: 'g' (protected left) and 'y'
%   (yellow) are both treated as non-red when computing same-state durations.
%   tg therefore includes yellow time. Use globalTLsInformation_Y instead
%   if yellow must be tracked separately.
%
%   Outputs:
%     tlAllRYGStates      - {phase, tlsIndex} state chars with 'g' normalised to 'G'
%     tlAllPhaseDurations - {phase, tlsIndex} duration (s) of each SUMO phase
%     tlAllSameStateDur   - {phase, tlsIndex} = [tg, tr]
%                           tg: total green block duration for this link at this phase (s)
%                           tr: total red block duration for this link at this phase (s)
%     tlCycleDur          - total signal cycle duration (s)
%
%   Lookup in the simulation loop (SUMO indices are 0-based, hence +1):
%     tg = tlAllSameStateDur{tlsPhase+1, tlsIndex+1}(1)
%     tr = tlAllSameStateDur{tlsPhase+1, tlsIndex+1}(2)

%% Query full TL programme from SUMO (done once — not repeated per step)
tlsRYGDefinition = traci.trafficlights.getCompleteRedYellowGreenDefinition(tlsID);
% tlsRYGState      = traci.trafficlights.getRedYellowGreenState(tlsID);
% tlsPhase         = traci.trafficlights.getPhase(tlsID);
% tlsPhaseDuration = traci.trafficlights.getPhaseDuration(tlsID);

%% Initialise
tlIndexPhaseStates              = {};
tlAllIndexStates                = {};
tlAllRYgGStates                 = {};
tlAllPhaseDurations             = {};
storeNextRYGchangePhase         = {};
storePhaseDurationUntilNextRYGChange = {};

tlscontrolledLanes = traci.trafficlights.getControlledLanes(tlsID);
tlscontrolledLinks = traci.trafficlights.getControlledLinks(tlsID);

numoftlIndexes = length(tlscontrolledLinks);
numofPhases    = length(tlsRYGDefinition{1,1}.phases);

%% Build state and duration matrices: [phase × tlsIndex]
% tlAllRYgGStates{i,j}     = raw state char for SUMO phase i, link index j
% tlAllPhaseDurations{i,j} = duration (s) of SUMO phase i (same for all j in a phase)

% Alternative index-wise layout (link × phase) — unused here, see globalTLsInformation_debug:
% for i = 1:numoftlIndexes
%     for j = 1:numofPhases
%         tlIndexPhaseStates{j} = tlsRYGDefinition{1}.phases{1,j}.state(i);
%     end
%     tlAllIndexStates{1,i} = tlIndexPhaseStates;
% end

for i = 1:numofPhases
    for j = 1:numoftlIndexes
        tlAllRYgGStates{i,j}    = tlsRYGDefinition{1}.phases{1,i}.state(j);
        tlAllPhaseDurations{i,j} = tlsRYGDefinition{1}.phases{1,i}.duration;
    end
end

%% Normalise 'g' (protected left-turn green) → 'G'
% This unifies all green variants so state comparisons only need to check
% for 'G' vs 'r'. Yellow ('y') is left as-is and will be absorbed into the
% green block duration during the same-state accumulation loop below.
tlAllRYGStates = strrep(tlAllRYgGStates, 'g', 'G');

%% Total signal cycle duration
tlCycleDur = sum(cell2mat(tlAllPhaseDurations(:,1)), 1);

%% Accumulate same-state block durations for every (phase, link) pair
% For each starting phase i and link j, walk forward through phases until
% the state changes, summing durations. This gives the total green (or red)
% block a vehicle would experience if it arrived at the start of phase i.
for j = 1:numoftlIndexes
    for i = 1:numofPhases

        currentPhase         = i;
        currentState         = tlAllRYGStates{i,j};
        currentPhaseDuration = tlAllPhaseDurations{i,j};
        prevState            = currentState;
        totalDuration        = 0;

        while prevState == currentState
            prevPhase         = currentPhase;
            prevState         = currentState;
            prevPhaseDuration = currentPhaseDuration;

            totalDuration = totalDuration + prevPhaseDuration;

            % Advance to next phase, wrapping at end of cycle.
            currentPhase = currentPhase + 1;
            if currentPhase > numofPhases
                currentPhase = currentPhase - numofPhases;
            end

            currentState         = tlAllRYGStates{currentPhase, j};
            currentPhaseDuration = tlAllPhaseDurations{currentPhase, j};
        end

        storeNextRYGchangePhase{i,j}             = currentPhase;  % 1-based index of next-different-state phase
        storePhaseDurationUntilNextRYGChange{i,j} = totalDuration;
    end
end

% Example of how to extract tg/tr for a specific vehicle (done inside main_SUMO.m):
% sameStateDuration  = storePhaseDurationUntilNextRYGChange{tlsPhase+1, tlsIndex+1};
% changeinWhichPhase = storeNextRYGchangePhase{tlsPhase+1, tlsIndex+1};
% oppStateDuration   = storePhaseDurationUntilNextRYGChange{changeinWhichPhase+1, tlsIndex+1};
% if tlsState == 'r'  % NOTE: == 'G'||'g' always true in MATLAB; test for 'r' instead
%     tr = sameStateDuration - tlElapsedTime;
%     tg = oppStateDuration;
% else
%     tg = sameStateDuration - tlElapsedTime;
%     tr = oppStateDuration;
% end

%% Assemble tlAllSameStateDur output table
% For each (phase, link) pair, store [tg, tr] using the un-normalised state
% char (tlAllRYgGStates, with 'g' still present) to classify the current phase.
% Yellow phases ('y') fall into the else branch and get tg = sameStateDuration.
for j = 1:numoftlIndexes
    for i = 1:numofPhases
        sameStateDuration  = storePhaseDurationUntilNextRYGChange{i,j};
        changeinWhichPhase = storeNextRYGchangePhase{i,j};
        oppStateDuration   = storePhaseDurationUntilNextRYGChange{changeinWhichPhase, j};

        if tlAllRYgGStates{i,j} == 'r'
            tr = sameStateDuration;
            tg = oppStateDuration;
        else  % 'G', 'g', or 'y' — all non-red treated as green
            tg = sameStateDuration;
            tr = oppStateDuration;
        end

        tlAllSameStateDur{i,j} = [tg, tr];
    end
end

end
