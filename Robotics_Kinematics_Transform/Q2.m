% input: f -> a 9-joint robot encoded as a SerialLink class
%        qInit -> 1x9 vector denoting current joint configuration
%        posGoal -> 3x1 vector denoting the target position to move to
% output: q -> 1x9 vector of joint angles that cause the end
%              effector position to reach <position>
%              (orientation is to be ignored)

function q = Q2(f, qInit, posGoal)
    % the initilization of q value
    q = qInit;
    
    stepsize = 0.01;
    error_threshold = 0.001;
    
    % the max time in iteration
    max_iter = 10000;
    iter = 1;
    
    % add the rotation elements
    
    posGoal = [posGoal; 0; 0; 0];
    while(iter < max_iter)
        iter = iter + 1;
        trans = f.fkine(q).t;
        % add the rotation elements
        x = [trans; 0; 0; 0];
        dx = posGoal - x;
        % for normalizing the error value
        error = norm(dx);
        if error < error_threshold
            break
        end
        jacobian = f.jacob0(q);
        jacobian_inv = pinv(jacobian' * jacobian) * jacobian';
        dq = stepsize .* jacobian_inv * dx;
        q = q + dq';
    end
end