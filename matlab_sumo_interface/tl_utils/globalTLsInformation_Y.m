function [tlAllIndexStates, tlAllPhaseDurations, tlAllSameStateDur] = globalTLsInformation_Y(tlsID)
%GLOBALTLSINFORMATION_Y  Pre-compute TL timing table with explicit yellow phase tracking.
%
%   NOT USED in the main ECO-AND OCP pipeline (main_SUMO.m) — provided as an
%   alternative to globalTLsInformation for scenarios where yellow duration
%   must be passed separately to the OCP solver.
%   Swap in via the commented-out call in main_SUMO.m:
%     [tlAllIndexStates, tlAllPhaseDurations, tlAllSameStateDur] = globalTLsInformation_Y(tlsID);
%
%   Key differences from globalTLsInformation:
%     1. tlAllSameStateDur{i,j} = [tg, tr, ty] — 3-element vector.
%        tg excludes yellow time; ty is stored separately.
%     2. Yellow phases are merged with green for same-state accumulation
%        ('y'→'G' normalisation in addition to 'g'→'G'), so the while-loop
%        accumulates the full green+yellow block. ty is then found separately
%        and subtracted from tg to give pure green duration.
%     3. Returns tlAllIndexStates (index-wise [link × phase] layout) in
%        addition to the phase-wise matrices. Not returned by globalTLsInformation.
%     4. No tlCycleDur output.
%
%   Outputs:
%     tlAllIndexStates    - {1, tlsIndex} cell of state-char arrays across all phases
%                           for each link index (index-wise layout)
%     tlAllPhaseDurations - {phase, tlsIndex} duration (s) of each SUMO phase
%     tlAllSameStateDur   - {phase, tlsIndex} = [tg, tr, ty]
%                           tg: pure green duration (excluding yellow) (s)
%                           tr: red block duration (s)
%                           ty: yellow phase duration (s)
%
%   NOTE: the closing 'end' of this function was commented out in the original
%   file. It has been restored here — do not comment it out again.

%% Query full TL programme from SUMO
tlsRYGDefinition = traci.trafficlights.getCompleteRedYellowGreenDefinition(tlsID);
% tlsRYGState      = traci.trafficlights.getRedYellowGreenState(tlsID);
% tlsPhase         = traci.trafficlights.getPhase(tlsID);
% tlsPhaseDuration = traci.trafficlights.getPhaseDuration(tlsID);

%% Initialise
tg = 0; tr = 0; ty = 0;
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

%% Build index-wise layout: tlAllIndexStates{1,j} across all phases for link j
for i = 1:numoftlIndexes
    for j = 1:numofPhases
        tlIndexPhaseStates{j} = tlsRYGDefinition{1}.phases{1,j}.state(i);
    end
    tlAllIndexStates{1,i} = tlIndexPhaseStates;
end

%% Build phase-wise layout: tlAllRYgGStates{i,j} & tlAllPhaseDurations{i,j}
for i = 1:numofPhases
    for j = 1:numoftlIndexes
        tlAllRYgGStates{i,j}    = tlsRYGDefinition{1}.phases{1,i}.state(j);
        tlAllPhaseDurations{i,j} = tlsRYGDefinition{1}.phases{1,i}.duration;
    end
end

%% Normalise: 'g' (protected left) → 'G', and 'y' (yellow) → 'G'
% Both yellow and protected-left are merged into green so the same-state
% accumulation loop below treats green+yellow as one continuous block.
% ty is extracted separately in the final loop and subtracted from tg.
tlAllRYGStates = strrep(tlAllRYgGStates, 'g', 'G');
tlAllRYGStates = strrep(tlAllRYGStates,  'y', 'G');  % <-- key difference from globalTLsInformation

%% Accumulate same-state block durations for every (phase, link) pair
% Because 'y' was merged into 'G', the totalDuration here captures the full
% green+yellow block. Yellow is peeled off in the final assembly loop below.
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

            currentPhase = currentPhase + 1;
            if currentPhase > numofPhases
                currentPhase = currentPhase - numofPhases;
            end

            currentState         = tlAllRYGStates{currentPhase, j};
            currentPhaseDuration = tlAllPhaseDurations{currentPhase, j};
        end

        storeNextRYGchangePhase{i,j}             = currentPhase;
        storePhaseDurationUntilNextRYGChange{i,j} = totalDuration;

        % Alternative: classify directly inside loop (unused — done in final loop below).
%         if tlAllRYgGStates{i,j} == 'r'
%             tr = totalDuration;
%         elseif tlAllRYgGStates{i,j} == 'y'
%             ty = totalDuration;
%         else
%             tg = totalDuration;
%         end
%         tlAllSameStateDur{i,j} = [tg, tr, ty];
    end
end

%% Assemble tlAllSameStateDur: [tg, tr, ty] per (phase, link)
% ty is the duration of the yellow phase for each link, found by locating
% the phase index where state == 'y' in the un-normalised state matrix.
for j = 1:numoftlIndexes

    % Find the yellow phase for this link and extract its duration.
    y_index = find([tlAllRYgGStates{:,j}] == 'y');
    ty = tlAllPhaseDurations{y_index, j};  % yellow duration (s)

    for i = 1:numofPhases
        sameStateDuration  = storePhaseDurationUntilNextRYGChange{i,j};
        changeinWhichPhase = storeNextRYGchangePhase{i,j};
        oppStateDuration   = storePhaseDurationUntilNextRYGChange{changeinWhichPhase, j};

        if tlAllRYgGStates{i,j} == 'r'
            tr = sameStateDuration;
            % Opposite block is green+yellow; deduct ty to get pure green.
            if tlAllRYgGStates{i,j} ~= 'y'   % always true when current phase is 'r'
                tg = oppStateDuration - ty;
            else
                tg = tlAllPhaseDurations{find([tlAllRYgGStates{:,j}] == 'G'), j};
            end
        else
            % Current phase is green (or yellow).
            if tlAllRYgGStates{i,j} ~= 'y'
                % Green phase: same-state block includes yellow; deduct ty.
                tg = sameStateDuration - ty;
            else
                % Yellow phase: tg is just the standalone green phase duration.
                tg = tlAllPhaseDurations{find([tlAllRYgGStates{:,j}] == 'G'), j};
            end
            tr = oppStateDuration;
        end

        tlAllSameStateDur{i,j} = [tg, tr, ty];
    end
end

end
