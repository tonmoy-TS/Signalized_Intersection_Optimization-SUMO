function plotSingleVehResults(test_vehicle)
%PLOTSINGLEVEHRESULTS  Compare OCP-commanded vs SUMO-executed speed for one vehicle.
%
%   Overlays two speed traces on a single figure:
%     - Blue line: speed actually executed in SUMO (from velAll, logged each step)
%     - Orange line: speed profile pre-computed by the OCP solver (from OCvel)
%
%   Used for post-run debugging — useful for verifying that the OCP profile
%   was replayed correctly by SUMO, or for diagnosing non-compliant vehicles.
%
%   Requires the following variables to exist in the caller's workspace
%   (populated during the main_SUMO.m simulation loop):
%     velAll       - cell array {step, vehicleIndex}: SUMO-reported speed (m/s)
%     OCvel        - cell array {step, vehicleIndex}: OCP-computed speed (m/s)
%     uniqueIDsAll - Map: vehicle ID string → column index in velAll
%     uniqueIDs    - Map: vehicle ID string → column index in OCvel
%
%   Input:
%     test_vehicle - vehicle ID string to plot (e.g. 'fWE.AV.2')
%
%   NOTE: the line below overrides the input argument with a hardcoded ID.
%   Comment it out and pass the desired ID as an argument instead.
test_vehicle = 'fWE.AV.2';

%% Earlier multi-channel plot (speed, acceleration, distance) — unused
% if plotNow == 1
%     figure; subplot(3,1,1);
%     plot(steps*0.1,vehSpeed);grid;  hold on;
%     plot(steps*0.1,vehSpeedWOTraci); hold on;
%     plot (steps*0.1, vehAllowedSpeed)
%     xlabel('Time (s)'); ylabel('Speed (m/s)');
%
%     subplot(3,1,2); plot(steps*0.1,vehAccel);grid;hold on;
%     subplot(3,1,2); plot(steps*0.1,vehDecel);
%     xlabel('Time (s)'); ylabel('Acceleration (m/s^2)');
%
%     subplot(3,1,3); plot(steps*stepSize,vehDistance);
%     grid; xlabel('Time(s)'); ylabel('Total Distance (m)');
% end

%% Plot executed vs OCP speed for the selected vehicle
figure;
plot(cell2mat(velAll(:, uniqueIDsAll(test_vehicle)))); hold on;  % SUMO-executed speed
%plot(cell2mat(velAllWOTraciplot(:,uniqueIDsAll(test_vehicle)))); hold on;  % without TraCI (reference)
plot(cell2mat(OCvel(:, uniqueIDs(test_vehicle))));                % OCP-computed speed
legend('Executed (SUMO)', 'OCP profile');
xlabel('Simulation step');
ylabel('Speed (m/s)');
title(['Speed profile: ' test_vehicle]);
grid on;

end
