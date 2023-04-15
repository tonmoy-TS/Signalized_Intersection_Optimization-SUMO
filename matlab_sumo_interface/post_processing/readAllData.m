function finalData = readAllData(output_folder, eco_thm, run_time)
%READALLDATA  Parse SUMO XML outputs, assemble metrics table, and plot results.
%
%   Called once at the end of main_SUMO.m after SUMO_Close(). Reads the XML
%   files that SUMO wrote during the simulation, combines lane traffic and
%   emission data into a single table keyed by MPR, generates a summary
%   figure, and returns the table for saving.
%
%   Expected files in output_folder (written by SUMO via edgeLaneAggregate.add.xml):
%     *laneDataDump.xml    — lane-level traffic metrics aggregated over the
%                            full simulation: waitingTime, laneDensity, speed
%     *laneEmissionDump.xml — lane-level emission metrics: fuel and CO2
%                             in absolute, normalised (per-metre), and
%                             per-vehicle forms
%
%   XML structure (both files):
%     <meandata>
%       <interval ...>
%         <edge id="...">
%           <lane id="..." waitingTime="..." laneDensity="..." speed="..." .../>
%         </edge>
%       </interval>
%     </meandata>
%   Parsed using Java DOM (xmlread); loop indices are 0-based (Java convention).
%
%   MPR ASSUMPTION: The output folder is expected to contain exactly 11 files
%   of each type — one per MPR level (0,10,20,...,100 %). The MPR values are
%   hardcoded in the order the files were produced (see MPR array below).
%   This is fragile: if runs are added, removed, or reordered, the MPR
%   assignment will be wrong. Consider embedding MPR in the filename and
%   parsing it dynamically if the sweep order changes.
%
%   Inputs:
%     output_folder - path to the timestamped results subfolder for this run
%     eco_thm       - optimisation objective string (for the info panel label)
%     run_time      - wall-clock duration of the simulation loop (s)
%
%   Output:
%     finalData - table with columns: MPR, waitingTime, Fuel Abs, Fuel Normed,
%                 Fuel Per Veh, CO2 Abs, CO2 Normed, CO2 Per Veh
%                 Sorted ascending by MPR (0 → 100 %)

global outputfoldername verbose
global envCons mpr vph range rho

folderPath = output_folder;

%% Discover XML output files
laneDataDumpxmlFiles     = dir(fullfile(folderPath, '*laneDataDump.xml'));
laneEmissionDumpxmlFiles = dir(fullfile(folderPath, '*laneEmissionDump.xml'));

%% Initialise collectors
laneId_laneData = {};
waitingTimes    = [];
laneDensities   = [];
speeds          = [];

laneId_emission = {};
fuelAbs    = [];
fuelNormed = [];
fuelPerVeh = [];
CO2Abs     = [];
CO2Normed  = [];
CO2PerVeh  = [];

%% Parse laneDataDump XML files
% Each file corresponds to one simulation run (one MPR level).
% getElementsByTagName traverses the full DOM tree, so 'lane' elements are
% found at any nesting depth — no need to walk edge/interval manually.
for i = 1:numel(laneDataDumpxmlFiles)
    laneDataxmlFile = xmlread(fullfile(folderPath, laneDataDumpxmlFiles(i).name));
    laneElements    = laneDataxmlFile.getElementsByTagName('lane');

    % Java DOM uses 0-based indexing; iterate from 0 to (length-1).
    for j = 0:laneElements.getLength - 1
        laneElement = laneElements.item(j);

        laneId      = char(laneElement.getAttribute('id'));
        waitingTime = str2double(char(laneElement.getAttribute('waitingTime')));   % (s)
        laneDensity = str2double(char(laneElement.getAttribute('laneDensity')));   % (veh/km)
        speed       = str2double(char(laneElement.getAttribute('speed')));         % (m/s)

        laneId_laneData = [laneId_laneData; laneId];
        waitingTimes    = [waitingTimes;    waitingTime];
        laneDensities   = [laneDensities;   laneDensity];
        speeds          = [speeds;          speed];
    end
end

% Intermediate table (unused — kept for reference if per-lane inspection needed):
% laneDataDump = table(laneId_laneData, waitingTimes, laneDensities, speeds, ...
%     'VariableNames', {'laneId', 'waitingTime', 'laneDensity', 'speed'});

%% Parse laneEmissionDump XML files
% Structure is deeper: interval > edge > lane. Walking explicitly to reach lane nodes.
for i = 1:numel(laneEmissionDumpxmlFiles)
    laneEmissionxmlDoc = xmlread(fullfile(folderPath, laneEmissionDumpxmlFiles(i).name));
    intervals = laneEmissionxmlDoc.getElementsByTagName('interval');

    for j = 0:intervals.getLength() - 1
        interval = intervals.item(j);
        edges    = interval.getElementsByTagName('edge');

        for k = 0:edges.getLength() - 1
            edge  = edges.item(k);
            lanes = edge.getElementsByTagName('lane');

            for l = 0:lanes.getLength() - 1
                lane = lanes.item(l);
                laneId_emission{end+1} = char(lane.getAttribute('id'));
                fuelAbs(end+1)    = str2double(lane.getAttribute('fuel_abs'));     % total fuel consumed (mL)
                fuelNormed(end+1) = str2double(lane.getAttribute('fuel_normed'));  % fuel per metre (mL/m)
                fuelPerVeh(end+1) = str2double(lane.getAttribute('fuel_perVeh')); % fuel per vehicle (mL/veh)
                CO2Abs(end+1)     = str2double(lane.getAttribute('CO2_abs'));      % total CO2 (mg)
                CO2Normed(end+1)  = str2double(lane.getAttribute('CO2_normed'));   % CO2 per metre (mg/m)
                CO2PerVeh(end+1)  = str2double(lane.getAttribute('CO2_perVeh'));   % CO2 per vehicle (mg/veh)
            end
        end
    end
end

% Intermediate table (unused — kept for reference):
% laneEmissionData = table(laneId_emission', fuelAbs', fuelNormed', fuelPerVeh', ...
%     CO2Abs', CO2Normed', CO2PerVeh', ...
%     'VariableNames', {'LaneID','FuelAbs','FuelNormed','FuelPerVeh','CO2Abs','CO2Normed','CO2PerVeh'});

%% Assign MPR values
% MPR levels are hardcoded in the order the runs appear in the output folder.
% The sweep in main_SUMO_simloop.m (or main_SUMO.m multi-run mode) produced files
% in this specific order: 60,70,80,30,40,50,90,100,0,10,20 %.
% Doubled to match the 22-row combined dataset (11 laneData + 11 emission rows).
% WARNING: this breaks silently if the run order or number of runs changes.
MPR = [60; 70; 80; 30; 40; 50; 90; 100; 0; 10; 20];
MPR = [MPR; MPR];  % 22 rows to match total lane entries across both XML types

%% Combine lane data and emission data into one table
combineData = table(laneId_laneData, MPR, waitingTimes, ...
    fuelAbs', fuelNormed', fuelPerVeh', CO2Abs', CO2Normed', CO2PerVeh', ...
    'VariableNames', {'laneId', 'MPR', 'waitingTime', ...
                      'Fuel Abs', 'Fuel Normed', 'Fuel Per Veh', ...
                      'CO2 Abs',  'CO2 Normed',  'CO2 Per Veh'});

% Take only the first 11 rows (one per MPR level from laneDataDump) and drop
% the laneId column — the emission rows are already reflected via the shared MPR column.
finalData = sortrows(combineData(1:11, 2:end), {'MPR'}, {'ascend'});

% Save the table to a CSV file (optional):
% writetable(finalData, 'finalData.csv');

%% Plot: 3×3 subplot grid — MPR vs lane metrics
% Layout:
%   [1,2]  waitingTime (spans two columns)   [3]  run info text
%   [4]    Fuel Abs    [5] Fuel Normed        [6]  Fuel Per Veh
%   [7]    CO2 Abs     [8] CO2 Normed         [9]  CO2 Per Veh
varNames = finalData.Properties.VariableNames;

fig = figure;

% Panel 1–2: waiting time (wide panel)
subplot(3, 3, 1:2);
plot(finalData.MPR, finalData.waitingTime);
xlabel('MPR (%)'); ylabel('Waiting time (s)');
title('Waiting time'); grid on;

% Panels 4–6: fuel metrics (varNames indices 3,4,5 = Fuel Abs, Normed, Per Veh)
for i = 3:5
    subplot(3, 3, i + 1);
    plot(finalData.MPR, finalData.(varNames{i}));
    xlabel('MPR (%)'); ylabel(varNames{i});
    xticks([0 10 20 30 40 50 60 70 80 90 100]);
    title(varNames{i}); grid on;
end

% Panels 7–9: CO2 metrics (varNames indices 6,7,8 = CO2 Abs, Normed, Per Veh)
for i = 6:8
    subplot(3, 3, i + 1);
    plot(finalData.MPR, finalData.(varNames{i}));
    xlabel('MPR (%)'); ylabel(varNames{i});
    xticks([0 10 20 30 40 50 60 70 80 90 100]);
    title(varNames{i}); grid on;
end

% Panel 3: run configuration info box
subplot(3, 3, 3);
text(0.1, 0.5, sprintf( ...
    'ECO-AND: %s\nSim duration: %0.0f s\nStep length: %0.2f s\nVPH: %d\nDSRC range: %d m\nRun time: %0.2f s', ...
    eco_thm, envCons.stepSize * envCons.numSteps, envCons.stepSize, vph, range, run_time));
axis off;

sgtitle('MPR vs Lane Data');
set(gcf, 'Position', [1, 49, 1536, 740]);  % maximise figure window

%% Save figure
figtitle = [output_folder, '/', 'vph_', num2str(vph), '-range_', num2str(range), '-MPR vs Lane data.png'];
saveas(fig, figtitle);

end
