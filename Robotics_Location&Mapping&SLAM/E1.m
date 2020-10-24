% Input: odo -> 2xT matrix containing odometry readings for T time steps
%        zind -> 1xT vector containing the observed landmark index for
%                T time steps; index is 0 if no landmark observed
%        z -> 1xT cell array containing the (range, bearing) observation
%             for T time steps; z{t} is empty if no observation at time t
%        V -> 2x2 matrix denoting the process noise in (forward, angular)
%        W -> 2x2 matrix denoting the sensing noise in (range, bearing)
%        x0 -> 3x1 vector denoting the initial vehicle state mean
%        P0 -> 3x3 matrix denoting the initial vehicle state covariance
%        map -> Robotics toolbox Map object containing the known map
%               (known landmarks) for localization
% Output: x_est -> 1xT cell array containing the vehicle state mean
%                  for T time steps (i.e., x_est{t} is a 3x1 vector)
%         P_est -> 1xT cell array containing the vehicle state covariance
%                  for T time steps (i.e., P_est{t} is a 3x3 matrix)

function [x_est, P_est] = E1(odo, zind, z, V, W, x0, P0, map)

% the total step number in the trajactory
traj_steps = size(z, 2);
for i = 1:traj_steps
    % get the location of pre-state
    % Here, X is pose in k-th step(or current state)
    if i == 1
        X = x0;
        P = P0;
    else
        X = X_esti;
        P = P_esti;
    end
    
    % the Prediction Step
    
    % get the k-th pose information -- x, y, and theta
    x = X(1,1); y = X(2,1); theta = X(3,1);
    % predict pose of (k+1)-th state(or next state)
    % the odometry values
    o_d = odo(1,i); o_t = odo(2, i);

    % Predict the Estimation of Pose
    x_pred = x + o_d*cos(theta);
    y_pred = y + o_d*sin(theta);
    theta_pred = theta + o_t;
    
    X_pred = [x_pred; y_pred; theta_pred];
    
    % Predict the Estimation of Covariance
    % calculate the Jacobian
    Fx = [1, 0, -o_d*sin(theta);0, 1, o_d*cos(theta); 0, 0, 1];
    Fv = [cos(theta), 0; sin(theta), 0; 0, 1];
    % get predicted covariance estimation 
    P_pred = Fx * P * Fx' + Fv * V * Fv';
    
    % Check whether to contimue to the Update state
    % if the observation is non-empty, then update !!
    if isempty(z{1, i}) == false
        % The Update Step
        x_i = map.map(1, zind(1,i));
        y_i = map.map(2, zind(1,i));

        h_r = sqrt((y_i - y_pred)^2 + (x_i - x_pred)^2);
        h_theta = atan2((y_i - y_pred), (x_i - x_pred));
        h_theta = angdiff_RoboTool(h_theta, theta_pred);
        z_esti = [h_r; h_theta];
        z_obs = z{1, i};
        innova = z_obs - z_esti;
        
        Hx = [-(x_i - x_pred)/h_r, -(y_i - y_pred)/h_r, 0; (y_i - y_pred)/(h_r^2), -(x_i - x_pred)/(h_r^2), -1];
        Hw = [1 0; 0 1];
        K = P_pred * Hx' / (Hx * P_pred * Hx' + Hw * W * Hw');

        X_esti = X_pred + K*innova;
        P_esti = P_pred - K * Hx * P_pred;
    else
        X_esti = X_pred;
        P_esti = P_pred;
    end
   
    x_est{1,i} = X_esti;
    P_est{1,i} = P_esti;
end

end