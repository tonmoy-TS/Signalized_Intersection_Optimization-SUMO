% MATLAB/SUMO implementation — Optimal Control of Vehicles Approaching a Traffic Light
% Ref.: Meng, X. et al. (2022) "Eco-Driving of Autonomous Vehicles for Nonstop
% Crossing of Signalized Intersections", IEEE Transactions on Automation Science and Engineering
% URL: https://ieeexplore.ieee.org/stamp/stamp.jsp?arnumber=9241792


function [OCvel, OCacc, fgr] = OC_SUMO(sgr, l, v_step, tg, tr, tlElapsedTime)
%OC_SUMO  Dispatch to the appropriate OCP solver and return a speed profile.
%
%   Inputs:
%     sgr          - current TL phase: 1 = green, 0 = red
%     l            - distance from vehicle to the stop line (m)
%     v_step       - vehicle speed at the moment it enters the control zone (m/s)  [= v0]
%     tg           - full green duration of the relevant TL phase (s)
%     tr           - full red duration of the relevant TL phase (s)
%     tlElapsedTime- time already elapsed within the current TL phase (s)  [= t0]
%
%   Outputs:
%     OCvel - velocity profile [v1; v2; ...] (m/s) for each future 1-s step
%     OCacc - corresponding acceleration profile (m/s²)
%     fgr   - free-green flag: 1 = vehicle can cruise at v0 and still hit green,
%                              0 = active speed adjustment was needed
%
%   The solver is selected by the global eco_thm:
%     'min-energy' → ECO_AND_min_energy  (rho_t=0, rho_u=1)
%     'min-time'   → ECO_AND_min_time    (rho_t=1, rho_u=0)
%     otherwise    → ECO_AND             (weighted, rho from OCAVTL)

global outputfoldername verbose
global envCons mpr vph range rho eco_thm

%% Physical limits (from Env_Const)
envCons = Env_Const();
vm = envCons.V_min;   % minimum speed (m/s)
vM = envCons.V_max;   % maximum speed (m/s)
aM = envCons.a_Max;   % max acceleration (m/s²)
am = envCons.a_min;   % max deceleration (m/s²)  [negative value]

v0 = v_step;          % rename for consistency with OCP notation
t0 = tlElapsedTime;   % elapsed time in current phase — starting time for OCP

%% State-space model of vehicle longitudinal dynamics
% Double-integrator: state x = [position; speed], input u = acceleration.
%   dx/dt = A*x + B*u,   y = C*x
% Used by lsim() inside the OCP solvers to simulate the resulting trajectory.
A = [0 1; 0 0];  B = [0; 1];
C = [1 0];       D = 0;
sys = ss(A, B, C, D);

%% Select and run the OCP solver
if contains(eco_thm, 'min-energy')
    % Minimum-energy: weight all cost on control effort (rho_u=1), none on time.
    rho_t = 0;  rho_u = 1;
    [t,u,x,y,J_Th,Jt,Ju,fgr] = ECO_AND_min_energy(rho_t,rho_u,tg,tr,t0,sgr,l,vm,v0,vM,am,aM,sys);

elseif contains(eco_thm, 'min-time')
    % Minimum-time: weight all cost on travel time (rho_t=1), none on energy.
    rho_t = 1;  rho_u = 0;
    [t,u,x,y,J_Th,Jt,Ju,fgr] = ECO_AND_min_time(rho_t,rho_u,tg,tr,t0,sgr,l,vm,v0,vM,am,aM,sys);

else
    % Weighted: balance energy vs time. rho is the time weight (0–1).
    % OCAVTL converts the scalar rho into (rho_t, rho_u) normalised to the
    % road length and speed range so the two cost terms are commensurate.
    rho = 0.9549;
    [rho_t, rho_u] = OCAVTL(l, vm, vM, aM, rho);
    [t,u,x,y,J_Th,Jt,Ju,fgr] = ECO_AND(rho_t,rho_u,tg,tr,t0,sgr,l,vm,v0,vM,am,aM,sys);
end

%% Extract velocity and acceleration profiles
% x is an (N×2) matrix from lsim: column 1 = position, column 2 = speed.
% Row 1 is the initial condition [0; v0], so the profile starts at row 2.
% u is a row vector; transpose and skip the first entry to match.
u_transpose = u';

if length(x) > 2
    OCvel = x(2:end, 2);        % speed at each future step (m/s)
    OCacc = u_transpose(2:end, 1);  % acceleration at each future step (m/s²)
else
    % OCP returned a trivially short solution (fgr=1 cruise, or solver failed).
    % Fall back to holding current speed; main_SUMO.m checks length(OCvel)<=2
    % alongside fgr==0 to identify genuine solver failures.
    OCvel = v0;
    OCacc = [];
end

end
