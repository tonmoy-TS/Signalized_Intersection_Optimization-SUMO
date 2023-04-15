function envCon_init = SUMO_Init()
%SUMO_INIT  Start SUMO, configure file paths, load constants, and subscribe.
%
%   Called once at the start of main_SUMO.m. Responsibilities:
%     1. Add the OCP solver folder (eco-and/) to the MATLAB path.
%     2. Build all input/output file path strings for the SUMO command line.
%     3. Launch sumo-gui via TraCI.
%     4. Load physical/environment constants (Env_Const).
%     5. Compute the junction bounding-box radius as a subscription buffer.
%     6. Subscribe to vehicle context data around the intersection.
%
%   Returns envCon_init: struct of environment constants (see Env_Const.m)
%   with a few extra fields appended here (delta_t, buffer, memeDetIDs).
%
%   PATH NOTE: All file paths below use '..\' relative to matlab_sumo_interface/.
%   This resolves to Signalized-Intersection-Optimization-SUMO/ one level up.
%   If you move the folder structure, update the root paths in the
%   "Input and output file paths" section below.

%% Global variables
global outputfoldername verbose
global envCons mpr vph range rho

%% Add subfolders to MATLAB path
% eco-and/        — OCP solvers (not distributed; obtain separately from Dr. Meng)
% post_processing/ — analysis and visualisation scripts (readAllData, plotTLsPhaseDiagram, etc.)
% tl_utils/       — traffic light timing utilities (globalTLsInformation, tlsInformation, etc.)
rootDir = fileparts(mfilename('fullpath'));
addpath(fullfile(rootDir, 'eco-and'));
addpath(fullfile(rootDir, 'post_processing'));
addpath(fullfile(rootDir, 'tl_utils'));

% -------------------------------------------------------------------------
% Earlier network configurations (kept for reference):
%     traci.start('sumo-gui -c ..\SUMO_network_files\sumo_files_tl-logic\free-flow.sumocfg --device.emissions.probability 1.0 --tripinfo-output trip_output.xml --start ');
%     traci.start('sumo-gui -c ..\SUMO_network_files\sumo_files_tl-logic\interfering-vehicles.sumocfg --device.emissions.probability 1.0 --tripinfo-output trip_output.xml --start ');
%     traci.start(['sumo-gui -c ..\SUMO_network_files\sumo_files_tl-logic\free-flow.sumocfg ' ...

%     traci.start(['sumo-gui -c ..\SUMO_network_files\sumo_files_tl-logic\interfering-vehicles.sumocfg '...
%                               '--additional-files ..\SUMO_network_files\sumo_files_tl-logic\e3-detectors.add.xml ' ...
%                               '--emission-output emissions_iv.xml ' ...
%                               '--output-prefix SUMO_results\TIME ' ...
%                               '--tripinfo-output trip_output.xml ' ...
%                               '--device.battery.explicit v_0 ' ...
%                               '--battery-output battery_output.xml ' ...
%                               '--edgedata-output edgedata.xml ' ...
%                               '--start ']);
% -------------------------------------------------------------------------

%% Input and output file paths
% IMPORTANT: Every argument string passed to traci.start must end with a
% trailing space ' ', otherwise SUMO concatenates adjacent arguments
% into a single unrecognised token and silently fails to parse them.
%
% PATH SEPARATORS (Windows vs Linux/Mac):
%   All paths below use Windows-style backslashes ('\') because they are
%   passed directly to the SUMO subprocess as a command-line string, not
%   through MATLAB's file I/O. Forward slashes ('/') were tested and did
%   not work reliably in this TraCI/SUMO setup on Windows.
%   If you are running on Linux or Mac, replace '\' with '/' in the path
%   strings on lines below (root1, e3detectors_files, edgeLane_files,
%   routes_files, and the traci.start call).
%
% FOLDER STRUCTURE ASSUMPTION:
%   Paths use '..\' relative to matlab_sumo_interface/, which assumes the
%   following sibling layout:
%     Signalized-Intersection-Optimization-SUMO/
%       matlab_sumo_interface/     <- MATLAB working directory when running
%       SUMO_network_files/
%         sumo_files_multilane-intersection/   <- active network
%         sumo_files_simple-intersection/
%         sumo_files_single-lane/
%         sumo_files_tl-logic/
%         sumo_files/
%         sumo_files_2/
%   If you move or rename any folder, update the paths accordingly.

%% Output prefix
% SUMO prepends this string to every output filename it writes (laneDataDump,
% laneEmissionDump, e3output, etc.). The literal token 'TIME' is replaced by
% SUMO with the simulation start timestamp.
% Path is relative from the SUMO working directory (matlab_sumo_interface/).
root1 = '..\matlab_sumo_interface\SUMO_results\';
root2 = outputfoldername;   % set by main_SUMO.m before calling SUMO_Init
% output_files = append(root1,root2,'TIME','-vph_',num2str(vph),'-range_',num2str(range),'-MPR_',num2str(mpr),'-',' '); % MPR-specific variant
output_files = append(root1,root2,'TIME','-vph_',num2str(vph),'-range_',num2str(range),'-',' ');

%% Additional (detector) files
% Two files are loaded as SUMO additional files:
%   e3Detectors_<range>m.add.xml  — defines E3 multi-entry-exit detectors
%                                    whose radius matches the DSRC range.
%                                    These set the AV control zone boundary.
%   edgeLaneAggregate.add.xml     — defines edge/lane data collection
%                                    (laneDataDump and laneEmissionDump outputs).
e3detectors_files = append('..\SUMO_network_files\sumo_files_multilane-intersection\e3Detectors_',num2str(range),'m.add.xml');
edgeLane_files    = '..\SUMO_network_files\sumo_files_multilane-intersection\edgeLaneAggregate.add.xml';
detectors_files   = append(e3detectors_files,',',edgeLane_files,' ');  % comma-separated list + trailing space

%% Route files
% Three route file variants are available (uncomment the one needed):
%
%   Single MPR value — one route file per (vph, mpr) combination.
%     Requires: route_files/vph<vph>/flow-vph_<vph>-MPR_<mpr>.rou.xml
% routes_files = append('..\SUMO_network_files\sumo_files_multilane-intersection\route_files\vph',num2str(vph),'\flow-vph_',num2str(vph),'-MPR_',num2str(mpr),'.rou.xml',' ');
%
%   Full MPR sweep (0–100 %) — a single route file contains flows at all MPR
%     levels, so one SUMO run covers the full MPR range at once. [ACTIVE]
routes_files = append('..\SUMO_network_files\sumo_files_multilane-intersection\route_files\flow-vph_',num2str(vph),'.rou.xml',' ');
%
%   Full MPR sweep — periodic/fixed vehicle count variant (probability-based).
% routes_files = append('..\SUMO_network_files\sumo_files_multilane-intersection\route_files\periodic_flow\flow-vph_',num2str(vph),'.rou.xml',' ');

%% Launch SUMO via TraCI
traci.start(['sumo-gui -c ..\SUMO_network_files\sumo_files_multilane-intersection\interfering-vehicles-no-turns.sumocfg ', ...
                '--output-prefix '    output_files   ...  % prefix for all SUMO output filenames
                '--additional-files ' detectors_files ...  % E3 detectors + lane aggregate collector
                '--route-files '      routes_files   ...  % vehicle demand / AV market penetration
                '--step-length 1 '                   ...  % simulation step = 1 s (must match Env_Const.stepSize)
                '--emissions.volumetric-fuel '       ...  % report fuel in litres (not mg/s)
                '--no-warnings true '                ...  % suppress SUMO warning output
                '--quit-on-end '                     ...  % close SUMO GUI when simulation finishes
                '--no-step-log true '                ...  % disable per-step console log (speeds up simulation)
                '--start ']);                              % start simulation immediately without manual play

% Optional SUMO flags (uncomment as needed):
%   '--random-depart-offset 0'           — disable random departure time jitter
%   '--step-length 1 '                   — (already active above)
%   '--queue-output queueinfo.xml '      — per-step queue lengths per lane
%   '--save-template template.xml '      — dump full config as XML template
%   '--aggregate-warnings 1 '            — collapse repeated warnings into one
%   '--no-step-log false '               — re-enable step counter in console
%   '--collision.stoptime 1 '            — stop colliding vehicle for 1 s
%   '--collision-output collisions.xml ' — log collision events
%   '--collision.mingap-factor 0 '       — disable min-gap collision check
%   '--collision.check-junctions '       — enable junction collision detection
%   '--collision.action warn '           — warn on collision instead of removing vehicle

%% Load environment constants
envCon_init = Env_Const();

% action_factor / state_factor: scaling values retained from an earlier
% reinforcement-learning version of this project; not used in the OCP path.
envCon_init.action_factor = max(abs(envCon_init.Max_Act), abs(envCon_init.Min_Act));
envCon_init.state_factor  = envCon_init.dist_to_TL;

% Confirm the actual SUMO step length matches the value in Env_Const.
envCon_init.delta_t = traci.simulation.getDeltaT();

% Compute junction bounding-box radius to use as a subscription buffer
% (see CalculateJnRadius below).
envCon_init.buffer = CalculateJnRadius(envCon_init.jn_ID);

envCon_init.memeDetIDs = traci.multientryexit.getIDList();

%% Subscribe to vehicle context around the intersection
SubscribeSumo(envCon_init.buffer);

% =========================================================================

    function SubscribeSumo(buffer)
    % Subscribe to vehicle variables within the detection radius of junction jn_ID.
    %
    % TraCI context subscriptions push updates every step for all vehicles
    % within (dist_to_TL + buffer) metres of the junction centroid, without
    % needing to call get* per vehicle. The buffer corrects for the offset
    % between the junction centroid position and the actual TL stop-line
    % positions at the junction boundary.
    %
    % Subscribed variables: VAR_TYPE, VAR_COLOR, VAR_LANE_ID.
    % (Speed and position are queried per vehicle in main_SUMO.m as needed.)

        import traci.constants
        sub_info = {constants.VAR_TYPE, constants.VAR_COLOR, constants.VAR_LANE_ID};
        traci.junction.subscribeContext(envCon_init.jn_ID,      ...
            constants.CMD_GET_VEHICLE_VARIABLE,                 ...
            envCon_init.dist_to_TL + buffer,                    ...
            sub_info);

        %traci.multientryexit.subscribe(envCon_init.memeDetIDs{1},...
        %                               {constants.LAST_STEP_MEAN_SPEED});
    end

% =========================================================================

    function buffer = CalculateJnRadius(jn_ID)
    % Compute the half-width of the junction as a subscription buffer.
    %
    % The junction centroid (getPosition) does not coincide with the TL
    % stop lines, which sit at the junction boundary. This function finds
    % the bounding box of the junction polygon and returns half its width
    % as the offset (buffer). Added to dist_to_TL, this ensures the
    % subscription radius reaches vehicles at the approach stop line.
    %
    % Assumption: the junction polygon is symmetric about its centroid.
    % If the geometry changes (e.g. asymmetric junction), this will return
    % buffer=0 and print an error — handle manually in that case.

        jn_POS   = traci.junction.getPosition(jn_ID);  % [x, y] centroid
        envCon_init.jn_POS = jn_POS;                   % store for reference
        jn_Shape = traci.junction.getShape(jn_ID);      % polygon vertices as cell array

        % Find the axis-aligned bounding box of the junction polygon.
        for i = 1:length(jn_Shape)
            if i == 1
                xmin = jn_Shape{i}(1); xmax = xmin;
                ymin = jn_Shape{i}(2); ymax = ymin;
                continue
            end
            xmin = min(xmin, jn_Shape{i}(1));
            xmax = max(xmax, jn_Shape{i}(1));
            ymin = min(ymin, jn_Shape{i}(2));
            ymax = max(ymax, jn_Shape{i}(2));
        end

        x = jn_POS(1); y = jn_POS(2);

        % Verify symmetry: all four half-extents should be equal (to 1 d.p.).
        if isequal(round((x-xmin),1), round((xmax-x),1), round((y-ymin),1), round((ymax-y),1))
            buffer = x - xmin;  % half-width = radius
        else
            buffer = 0;
            disp('ERROR! NEED TO HANDLE JUNCTION GEOMETRY')
        end
    end

end
