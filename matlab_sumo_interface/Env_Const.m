function envCons = Env_Const()
%ENV_CONST  Return a struct of environment and vehicle constants.
%
%   Called by SUMO_Init at startup; also called inside OC_SUMO each time
%   a new vehicle is processed (to ensure a clean parameter set).
%
%   Fields are grouped below by purpose:
%     1. Vehicle speed & acceleration limits  — used by all OCP solvers
%     2. Simulation time parameters           — used by main_SUMO.m
%     3. SUMO network identifiers             — must match the .net.xml file
%     4. Context subscription radius          — used by SUMO_Init
%     5. Legacy RL fields                     — not used in the OCP pipeline

    %% 1. Vehicle speed and acceleration limits
    % Bounds enforced by the OCP solvers (FiTTOCA, FiTTOCD, FTTOC).
    % Values match the SUMO vClass="passenger" defaults.
    envCons.V_max  = 24.6;   % maximum speed (m/s)  ≈ 88.6 km/h
    envCons.V_min  = 0;      % minimum speed (m/s)  — vehicle at rest
    envCons.a_Max  = 2.6;    % maximum acceleration (m/s²)  — SUMO passenger default
    envCons.a_min  = -4.5;   % maximum deceleration (m/s²)  — SUMO passenger default (negative)

    %% 2. Simulation time parameters
    envCons.stepSize    = 1.0;   % simulation step length (s) — must match --step-length in SUMO_Init
    envCons.numSteps    = 3600;  % total simulation steps (3600 s = 1 hour at 1 s/step)

    %% 3. SUMO network identifiers
    % These must exactly match the IDs defined in the .net.xml network file.
    envCons.jn_ID    = 'jnCent';  % junction ID of the controlled intersection
    envCons.ego_type = 'ev-ego';  % vehicle type ID for the single ego vehicle (legacy RL — unused in OCP pipeline)

    %% 4. Context subscription radius
    % dist_to_TL is the radius (m) around jn_ID within which vehicles are
    % tracked via the TraCI context subscription (see SUMO_Init/SubscribeSumo).
    % Set larger than the DSRC range (300 m) so vehicles are already in the
    % subscription before they reach the E3 detector boundary.
    envCons.dist_to_TL = 1000;  % context subscription radius (m)

    % t0: elapsed time in the current TL phase — placeholder only.
    % Overwritten by tlElapsedTime from TraCI in OC_SUMO before any solver call.
    envCons.t0 = 0;

    %% 5. Legacy RL fields (not used in the OCP pipeline)
    % Retained for compatibility with SUMO_Reset.m and SUMO_Step.m.

    % Initial speed distribution for RL episode resets (SUMO_Reset.m).
    % Note: units appear to be km/h here (inconsistent with V_max in m/s) —
    % verify before re-enabling the RL pipeline.
    envCons.v0_center  = 45;   % mean of random initial speed distribution
    envCons.v0_range   = 15;   % uniform spread around v0_center

    % RL agent action bounds (speed commands, m/s).
    envCons.Max_Act = envCons.V_max;  % upper speed command limit
    envCons.Min_Act = envCons.V_min;  % lower speed command limit

    % FreeFlowingSimple: controls step-mode branching in SUMO_Step.m.
    envCons.FreeFlowingSimple = true;

    % RL reward / penalty weights (SUMO_Step.m).
    envCons.rho_C  = -60;      % constraint-violation penalty (e.g. speed < 0 or > V_max)
    envCons.rho_e  = 0.6;      % weight on energy consumption term
    envCons.rho_v  = 0.2;      % weight on speed-deviation term
    envCons.rho_d  = 1/1000;   % weight on distance-travelled term

    % Maximum RL training episodes.
    envCons.numEpisodes = 2000;

end
