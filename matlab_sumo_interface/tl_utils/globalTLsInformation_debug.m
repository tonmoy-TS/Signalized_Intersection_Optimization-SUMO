%% globalTLsInformation_debug — development/debug script version of globalTLsInformation
%
%   NOT USED in the main ECO-AND OCP pipeline (main_SUMO.m).
%   NOT A FUNCTION — the function declaration and closing 'end' are both
%   commented out, so this runs as a script in the caller's workspace.
%   Variables tlsID, tlsRYGDefinition, etc. must already exist there.
%
%   Differences from globalTLsInformation.m:
%     1. Makes additional live TraCI queries at call time:
%          getRedYellowGreenState, getPhase, getPhaseDuration
%        (all commented out in the active version to avoid per-step overhead).
%     2. Builds tlAllIndexStates: an alternative [link × phase] layout
%        complementary to the [phase × link] layout of tlAllRYgGStates.
%        tlAllIndexStates{1,j} = cell array of state chars across all phases
%        for link index j. Useful for querying "what phases is link j green?".
%     3. No tlCycleDur output.
%     4. tlAllSameStateDur{i,j} = [tg, tr] — same 2-element format as
%        globalTLsInformation (yellow folded into green).
%
%   To convert into a proper function, uncomment the function declaration
%   and the closing 'end' below.

% function [tlAllIndexStates, tlAllPhaseDurations, tlAllSameStateDur] = globalTLsInformation_debug(tlsID)

%% Query full TL programme from SUMO
tlsRYGDefinition  = traci.trafficlights.getCompleteRedYellowGreenDefinition(tlsID);
tlsRYGState       = traci.trafficlights.getRedYellowGreenState(tlsID);   % current combined state string
tlsPhase          = traci.trafficlights.getPhase(tlsID);                  % current phase index (0-based)
tlsPhaseDuration  = traci.trafficlights.getPhaseDuration(tlsID);          % current phase total duration (s)

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

%% Build index-wise layout: tlAllIndexStates{1,j} = states across all phases for link j
% This layout complements the phase-wise matrix below.
% Each cell tlAllIndexStates{1,j} is a 1×numofPhases cell of state chars,
% letting callers ask "which phases is link j green?" without slicing a matrix.
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

%% Normalise 'g' (protected left-turn) → 'G'; yellow stays as 'y'
tlAllRYGStates = strrep(tlAllRYgGStates, 'g', 'G');

%% Accumulate same-state block durations for every (phase, link) pair
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
    end
end

% Example lookup (same pattern as globalTLsInformation):
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

%% Assemble tlAllSameStateDur: [tg, tr] per (phase, link)
for j = 1:numoftlIndexes
    for i = 1:numofPhases
        sameStateDuration  = storePhaseDurationUntilNextRYGChange{i,j};
        changeinWhichPhase = storeNextRYGchangePhase{i,j};
        oppStateDuration   = storePhaseDurationUntilNextRYGChange{changeinWhichPhase, j};

        if tlAllRYgGStates{i,j} == 'r'
            tr = sameStateDuration;
            tg = oppStateDuration;
        else
            tg = sameStateDuration;
            tr = oppStateDuration;
        end

        tlAllSameStateDur{i,j} = [tg, tr];
    end
end

% end
