% Input: V -> 2x2 matrix denoting the process noise in (forward, angular)
%        W -> 2x2 matrix denoting the sensing noise in (range, bearing)
%        x0 -> 3x1 vector denoting the initial vehicle state mean
%        P0 -> 3x3 matrix denoting the initial vehicle state covariance
%        range -> Scalar denoting the maximum range of the sensor
%        fov -> 1x2 vector denoting the [min_bearing, max_bearing]
%               (field of view) that can be detected by the sensor
% Output: ekf_l -> Robotics toolbox EKF object
%                  after localizing against a known map
%         ekf_m -> Robotics toolbox EKF object
%                  after estimating a map with known vehicle poses
%         ekf_s -> Robotics toolbox EKF object after performing SLAM

function [ekf_l, ekf_m, ekf_s] = E0(V, W, x0, P0, range, fov)

    % Localization
    % ekf_l 
    rng(0);
    map = LandmarkMap(20);
    veh = Bicycle('covar', V);
    veh.x0 = [6; 0; pi/2];
    D = RandomPath(map.dim);
    veh.add_driver(RandomPath(map.dim));
    sensor = RangeBearingSensor(veh, map, 'covar', W, 'angle', 	fov, 'range', range, 'animate');
    disp(range);
    disp(fov);
    ekf_l = EKF(veh, V, P0, sensor, W, map);
    ekf_l.run(1000);
    
    % After creating a map and vehicle, and running the Robotics toolbox
    % EKF, running the following line should produce Figure 1.
    % If it does not, you may have forgotten to reset the random seed to 0.
    % visualize({}, {}, [], map, veh, 'n');
    
    % Mapping
    rng(0);
    veh = Bicycle(); % based on assumption: No Pose Error
    veh.add_driver(RandomPath(map.dim));
    W = diag([0.1, 1*pi/180].^2);
    sensor = RangeBearingSensor(veh, map, 'covar', W, 'angle', 	fov, 'range', range, 'animate');
    ekf_m = EKF(veh, [], [], sensor, W, []);
    ekf_m.run(1000);
    
    % SLAM
    rng(0);
    veh = Bicycle('covar', V);
    veh.add_driver(RandomPath(map.dim));
    sensor = RangeBearingSensor(veh, map, 'covar', W, 'angle', 	fov, 'range', range, 'animate');
    ekf_s = EKF(veh, V, P0, sensor, W, []);
    ekf_s.run(1000);
    
end