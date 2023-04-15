function [tg, tr, ty] = tlsInformation(tlsID, tlsPhase, tlsIndex, tlsState, tlsTimeToEnd)
%TLSINFORMATION  Get green and red durations for a specific vehicle's TL situation.
%
%   SUPERSEDED — not used in the main ECO-AND OCP pipeline (main_SUMO.m).
%   This is the per-vehicle, per-step predecessor to globalTLsInformation.m.
%   It was replaced because it calls getCompleteRedYellowGreenDefinition on
%   every invocation, which is an expensive TraCI query. globalTLsInformation
%   makes that query once at startup and caches the full phase table instead.
%
%   The core algorithm is identical to globalTLsInformation: for the given
%   (tlsPhase, tlsIndex) pair, walk forward through TL phases until the
%   state changes, summing durations to get the total same-state duration;
%   then look up the opposite-state duration from the next block of phases.
%
%   Inputs:
%     tlsID        - TraCI traffic light controller ID string
%     tlsPhase     - current phase index (0-based, as returned by TraCI)
%     tlsIndex     - TL link index for this vehicle's approach (0-based)
%     tlsState     - current phase state character: 'G', 'g', 'r', or 'y'
%     tlsTimeToEnd - seconds remaining until the next phase switch
%
%   Outputs:
%     tg - full green duration for this TL link (s)
%     tr - full red duration for this TL link (s)
%     ty - yellow duration (always 0 here — yellow is not separately tracked)
%
%   NOTE: tg/tr are full cycle durations, not remaining time in the current
%   phase. tlElapsedTime is hardcoded to 0 below; see the commented line for
%   the alternative that deducts elapsed time to give remaining duration.

% tlsProgram = traci.trafficlights.getProgram(tlsID);

%% Query full TL programme from SUMO
% NOTE: this TraCI call is made on every invocation — expensive if called
% each step per vehicle. Use globalTLsInformation.m to cache this instead.
tlsRYGDefinition   = traci.trafficlights.getCompleteRedYellowGreenDefinition(tlsID);
% tlsRYGState        = traci.trafficlights.getRedYellowGreenState(tlsID);
% tlsPhase           = traci.trafficlights.getPhase(tlsID);
% tlsPhaseDuration   = traci.trafficlights.getPhaseDuration(tlsID);
tlscontrolledLanes = traci.trafficlights.getControlledLanes(tlsID);
tlscontrolledLinks = traci.trafficlights.getControlledLinks(tlsID);

numoftlIndexes = length(tlscontrolledLinks);
numofPhases    = length(tlsRYGDefinition{1,1}.phases);

%% Build state and duration matrices indexed by [phase, tlsIndex]
% tlAllRYGStates{i,j}    = state character for phase i, link index j
% tlAllPhaseDurations{i,j} = duration (s) of phase i (same for all j within a phase)

% %% Index-wise states for all phases (alternative layout — unused)
% for i = 1:numoftlIndexes
%     for j = 1:numofPhases
%         tlIndexPhaseStates{j} = tlsRYGDefinition{1}.phases{1,j}.state(i);
%     end
%     allIndexStates{1,i} = tlIndexPhaseStates;
% end

for i = 1:numofPhases
    for j = 1:numoftlIndexes
        tlAllRYgGStates{i,j}    = tlsRYGDefinition{1}.phases{1,i}.state(j);
        tlAllPhaseDurations{i,j} = tlsRYGDefinition{1}.phases{1,i}.duration;
    end
end

% Normalise protected-left 'g' → 'G' so green comparisons are uniform.
tlAllRYGStates = strrep(tlAllRYgGStates, 'g', 'G');

%% Compute same-state duration for every (phase, link index) pair
% For each starting phase i and link j, walk forward through successive
% phases until the state changes, accumulating total duration.
% This gives the full green (or red) block duration seen by a vehicle that
% arrives exactly at the start of phase i on link j.
for j = 1:numoftlIndexes
    for i = 1:numofPhases

        currentPhase        = i;
        currentState        = tlAllRYGStates{i,j};
        currentPhaseDuration = tlAllPhaseDurations{i,j};
        prevState           = currentState;
        totalDuration       = 0;

        while prevState == currentState
            prevPhase        = currentPhase;
            prevState        = currentState;
            prevPhaseDuration = currentPhaseDuration;

            totalDuration = totalDuration + prevPhaseDuration;

            % Advance to next phase, wrapping around at end of cycle.
            currentPhase = currentPhase + 1;
            if currentPhase > numofPhases
                currentPhase = currentPhase - numofPhases;
            end

            currentState        = tlAllRYGStates{currentPhase, j};
            currentPhaseDuration = tlAllPhaseDurations{currentPhase, j};
        end

        % Store the phase index where the state changes (0-based offset applied).
        % NOTE: uses currentPhase-1 here, unlike globalTLsInformation.m which
        % stores currentPhase directly. This difference affects the +1 offset
        % needed when reading back from the table.
        storeNextRYGchangePhase{i,j}            = currentPhase - 1;
        storePhaseDurationUntilNextRYGChange{i,j} = totalDuration;
    end
end

%% Extract tg and tr for the current vehicle's (phase, index) pair
% +1 on both indices converts from SUMO's 0-based to MATLAB's 1-based indexing.
sameStateDuration = storePhaseDurationUntilNextRYGChange{tlsPhase+1, tlsIndex+1};

% Identify the phase where the state flips, then look up its duration block.
changeinWhichPhase = storeNextRYGchangePhase{tlsPhase+1, tlsIndex+1};
oppStateDuration   = storePhaseDurationUntilNextRYGChange{changeinWhichPhase+1, tlsIndex+1};

% tlElapsedTime = 0 means tg/tr are full cycle durations (OCP uses them this way).
% Uncomment the line below to deduct elapsed time and get *remaining* duration instead.
tlElapsedTime = 0;
% tlElapsedTime = tlAllPhaseDurations{tlsPhase+1, tlsIndex+1} - tlsTimeToEnd;

% Assign tg/tr based on current state.
% NOTE: comparing against 'r' directly because tlsState=='G'||'g' is always
% true in MATLAB for non-empty char (see same note in main_SUMO.m).
if tlsState == 'r'
    tr = sameStateDuration - tlElapsedTime;  % remaining (or full) red
    tg = oppStateDuration;                   % full next green
    ty = 0;
else
    tg = sameStateDuration - tlElapsedTime;  % remaining (or full) green
    tr = oppStateDuration;                   % full next red
    ty = 0;
end

% -------------------------------------------------------------------------
% Stub helper functions from early development (unused, retained for context):
%
%   getPhaseDuration(tlsPhase, tlsIndex)
%     — would compute same-state duration starting from a given phase.
%
%   nextStateChangePhase(currentPhase, tlsIndex)
%     — would return the phase index at which state next changes.
%
%   currentSameStateDuration(currentPhase, tlsIndex)
%     — would return the accumulated same-state duration matrix.
%
% These were folded into the nested loop above.
% -------------------------------------------------------------------------

end
