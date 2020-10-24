% Input: odo -> 2xT matrix containing odometry readings for T time steps
%        zind -> 1xT vector containing the observed landmark index for
%                T time steps; index is 0 if no landmark observed
%        z -> 1xT cell array containing the (range, bearing) observation
%             for T time steps; z{t} is empty if no observation at time t
%        W -> 2x2 matrix denoting the sensing noise in (range, bearing)
%        x0 -> 3x1 vector denoting the known initial vehicle state
% Output: x_est -> 1xT cell array containing the map state mean
%                  for T time steps (i.e., x_est{t} is a (2M)x1 vector,
%                  where M is the number of landmarks observed by time t)
%         P_est -> 1xT cell array containing the vehicle state covariance
%                  for T time steps (i.e., P_est{t} is a (2M)x(2M) matrix,
%                  where M is the number of landmarks observed by time t)
%         indices -> Mx1 vector containing the landmark index corresponding
%                    to the entries in the state vector, where M is the
%                    number of landmarks observed by the final time step T)
%                    For example, if indices is [15; 4], then the first two
%                    rows of x_est and P_est correspond to landmark 15,
%                    and the next two rows correspond to landmark 4, etc.

function [x_est, P_est, indices] = E2(odo, zind, z, W, x0)

indices = [];
M = 0;
% the initial Landmarks Vector
X_esti = [];
% the initial landmarks Covariance
P_esti = [];

% the total step number in the trajactory
traj_steps = size(z, 2);
for i = 1:traj_steps
    % get the location of pre-state
    % Here, robo_X is pose if robot in k-th step(or current state)
    if i == 1
        robo_X = x0;
    end
    
    % the Prediction Step
    
    % Get the Pose information of Robot in next step
    % get the k-th pose information -- x, y, and theta
    x = robo_X(1,1); y = robo_X(2,1); theta = robo_X(3,1);
    % predict pose of (k+1)-th state(or next state)
    % the odometry values
    o_d = odo(1,i); o_t = odo(2, i);
    % Next Pose of Robot after one step forward -- K+1
    x_next = x + o_d * cos(theta);
    y_next = y + o_d * sin(theta);
    theta_next = theta + o_t;
    
    robo_X = [x_next; y_next; theta_next];
    
    % Predict the Pose and Covariance of landmarks
    X_pred = X_esti;
    P_pred = P_esti;
    
    % Check whether to contimue to the Update state
    % if the observation is non-empty, then update !!
    if isempty(z{1, i}) == false
        % The Update Step
        
        % Keep track of alignment
        landmark_idx = find(indices == zind(1,i));
        if isempty(landmark_idx)
            indices = [indices; zind(1,i)];

            % obtain the observation Z information
            z_obs = z{1, i};
            r = z_obs(1); beta = z_obs(2);

            % Inverse H to get position of landmar
            g_x = x_next + r * cos(theta_next + beta);
            g_y = y_next + r * sin(theta_next + beta);

            % Update the X estimation
            X_esti = [X_pred; g_x; g_y];
        
            % Update the covariance
            G_z = [cos(robo_X(3,1) + beta), -r*sin(robo_X(3,1) + beta); sin(robo_X(3,1) + beta), r*cos(robo_X(3,1) + beta)];
            % G_x are all zeros so we just create 0(2*n) to replace [G_x, 0(2*n-3)]
            Y_z = [eye(2*M), zeros(2*M, 2); zeros(2, 2*M), G_z];
            P_temp = [P_pred, zeros(2*M, 2);zeros(2, 2*M), W];
            P_esti = Y_z * P_temp * Y_z';
        
            % increase the number of Landmarks observed
            M = M + 1;
        else
            % Update Steps
            z_obs = z{1, i};
            g_x = X_pred(landmark_idx*2 - 1, 1);
            g_y = X_pred(landmark_idx*2, 1);
            
            % esitmated observing about veh with respect of landmark
            h_r = sqrt((g_y - y_next)^2 + (g_x - x_next)^2);
            h_theta = atan2((g_y - y_next), (g_x - x_next));
            h_theta = angdiff_RoboTool(h_theta, theta_next);
            z_esti = [h_r; h_theta];
            % compute innovation
            innova = z_obs - z_esti;
            
            % Compute the kalman filter
            H_pi = [(g_x - x_next)/h_r, (g_y - y_next)/h_r; -(g_y - y_next)/(h_r^2), (g_x - x_next)/(h_r^2)];
            Hx = [zeros(2, 2*(landmark_idx-1)), H_pi, zeros(2, 2*(M - landmark_idx))];
            Hw = [1 0; 0 1];
            K = P_pred * Hx' / (Hx * P_pred * Hx' + Hw * W * Hw');
            
            % Update landmark pose and covarianve
            X_esti = X_pred + K*innova;
            P_esti = P_pred - K * Hx * P_pred;
        end
        
    else
        X_esti = X_pred;
        P_esti = P_pred;
    end
   
    x_est{1,i} = X_esti;
    P_est{1,i} = P_esti;
end

end