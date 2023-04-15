%% ECO-AND parameter sweep launcher
%   Runs main_SUMO.m repeatedly across combinations of vph and DSRC range.
%   Each call to main_SUMO.m is a complete 1-hour SUMO simulation; results
%   are saved to a timestamped subfolder in SUMO_results/ by main_SUMO.m.
%
%   Two sweep modes are provided (see below):
%     1. MPR sweep  — fix vph & range, iterate over market penetration rate
%     2. vph/range sweep — fix eco_thm, iterate over traffic demand & detection range
%
%   Author: Tonmoy Sarker

clear all; close all; clc; format compact
import traci.constants

% Globals must be declared here before run("main_SUMO.m") executes, because
% main_SUMO.m writes to them and MATLAB requires global declarations in every
% workspace that accesses a global variable.
global outputfoldername verbose
global envCons mpr vph range rho

%% Mode 1: MPR sweep (fixed vph & range, iterate mpr = 20 / 40 / 60 / 80 / 100 %)
% Requires MPR-specific route files:
%   sumo_files_multilane-intersection/route_files/flow-vph_<vph>-MPR_<mpr>.rou.xml
% Uncomment this block and comment out Mode 2 to use.
% count=1;
% vph = 800;
% range = 300;
%
% for i=2:6
%     mpr = (i-1)*20;   % mpr = 20, 40, 60, 80, 100
%     fprintf('(%d): vph: %d, mpr: %d, range: %d.\n',count, vph,mpr,range);
%     run("main_SUMO.m")
%     clearvars -except mpr i vph range count   % clear main_SUMO.m workspace debris
%     i=i+1;
%     count =count+1;
% end

%% Mode 2: vph / range sweep (active)
% Sweeps over DSRC detection range and traffic demand.
%   range_i = 2 → range = 300 m
%   range_i = 3 → range = 450 m
%   vph_i   = 2 → vph   = 400 veh/h/lane
%
% To extend the sweep, adjust the loop bounds:
%   range: range_i = 1:3 → {150, 300, 450} m
%   vph:   vph_i   = 1:4 → {200, 400, 600, 800} veh/h/lane
count = 1;
for range_i = 2:3
    range = range_i * 150;

    for vph_i = 2:2
        vph = vph_i * 200;

        fprintf('(%d): vph: %d, range: %d.\n', count, vph, range);
        run("main_SUMO.m")

        % main_SUMO.m leaves its entire workspace behind after run().
        % clearvars resets everything except the loop counters so each
        % iteration of main_SUMO.m starts from a clean state.
        clearvars -except vph vph_i range range_i count
        count = count + 1;
        vph_i = vph_i + 1;    % redundant with for-loop increment, but harmless
    end
    range_i = range_i + 1;    % redundant with for-loop increment, but harmless
end
