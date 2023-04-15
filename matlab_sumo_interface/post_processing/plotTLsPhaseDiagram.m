function plotTLsPhaseDiagram(tlAllRYGStates, tlAllPhaseDurations)
%PLOTTLSPHASEDIAGRAM  Draw a time-space TL phase diagram (green/red bands).
%
%   Plots horizontal green and red bands for each straight-through TL link
%   over the full simulation duration. Each band sits at a fixed Y position
%   corresponding to an approach distance from the junction, giving a
%   time-space diagram that shows when each arm is green or red.
%
%   Only straight-through (edge) links are plotted — the loop steps by 3
%   (i = 1, 4, 7, 10) to skip the left-turn and right-turn link indices
%   within each arm. This assumes 3 controlled links per arm (straight,
%   left, right) at a 4-arm intersection.
%
%   Inputs:
%     tlAllRYGStates       - {phase, tlsIndex} cell array of state chars ('G','r','y')
%                            as returned by globalTLsInformation
%     tlAllPhaseDurations  - {phase, tlsIndex} cell array of phase durations (s)
%                            as returned by globalTLsInformation
%
%   Requires envCons (global) for stepSize and numSteps.
%
%   Called optionally from main_SUMO.m after the simulation — uncomment the
%   plotTLsPhaseDiagram call and the velAll/distAll logging block to use.

import traci.constants
global outputfoldername verbose
global envCons mpr vph range rho

stepSize = envCons.stepSize;
numSteps = envCons.numSteps;
simDur   = stepSize * numSteps;  % total simulation duration (s)
% simDur = 300;  % override to zoom into the first 300 s

% Y-axis baseline offset (m). Each link's band is plotted at dist + i*5,
% so bands are vertically separated by 5 m per link index step.
dist = 500;

%% Plot green/red phase bands for each straight-through link
% i steps through link indices 1, 4, 7, 10 (one per arm, skipping turns).
for i = 1:3:12

    if tlAllRYGStates{1, i} == 'G'
        % Phase 1 is green for this link: cycle starts with green then red.
        tg = tlAllPhaseDurations{1, i};
        tr = tlAllPhaseDurations{2, i};
        n  = floor(simDur / (tg + tr));  % number of complete cycles in simDur

        for k = 0:n+1
            if k * (tr + tg) >= simDur
                break  % do not draw beyond the simulation end
            end
            % Green band: from start of cycle to end of green phase.
            plot([k*(tr+tg),  k*(tr+tg)+tg],  [dist+i*5, dist+i*5], 'green', 'LineWidth', 2); hold on
            axis([0 simDur 0 inf])
            % Red band: from end of green to start of next cycle.
            plot([k*(tr+tg)+tg, (k+1)*(tr+tg)], [dist+i*5, dist+i*5], 'red',   'LineWidth', 2); hold on
            axis([0 simDur 0 inf])
        end

    else
        % Phase 1 is red for this link: cycle starts with red then green.
        tr = tlAllPhaseDurations{1, i};
        tg = tlAllPhaseDurations{2, i};
        n  = floor(simDur / (tg + tr));

        for k = 1:n+1
            % Red band: from start of cycle to end of red phase.
            plot([(k-1)*(tr+tg),    (k-1)*(tr+tg)+tr], [dist+i*5, dist+i*5], 'red',   'LineWidth', 2); hold on
            axis([0 simDur 0 inf])
            % Green band: from end of red to start of next cycle.
            plot([(k-1)*(tr+tg)+tr, k*(tr+tg)],        [dist+i*5, dist+i*5], 'green', 'LineWidth', 2); hold on
            axis([0 simDur 0 inf])
        end
    end

    i = i + 1;  % redundant — for-loop already increments i; harmless

end

xlabel('Time (s)');
ylabel('Distance from junction (m)');
title('Traffic Light Phase Diagram');

end
