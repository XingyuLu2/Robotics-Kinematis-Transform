% input: f -> a 9-joint robot encoded as a SerialLink class
%        qInit -> 1x9 vector denoting current joint configuration
%        posGoal -> 3x1 vector describing goal position
%        epsilon -> scalar denoting how close end effector must be before
%                   loop terminates
%        velocity -> scalar denoting desired velocity of end effector
% output: traj -> nx9 matrix that denotes arm trajectory. Each row denotes
%                 a joint configuration. The first row is the first
%                 configuration. The last is the last configuration in the
%                 trajectory.

function traj = Q3(f, qInit, posGoal, epsilon, velocity)
    q = qInit;
    % the initial position of the end_effect
    % and then compute the total distance
    posInit = f.fkine(qInit).t;
    distance = sqrt(sum((posGoal-posInit).^2));
    
    iter = 0;
    traj = [q];
    step_forward = (posGoal - posInit) / distance;
    step_forward = [step_forward; 0; 0; 0];
    % add the rotatin elements
    posGoal = [posGoal; 0; 0; 0];
    while(distance > epsilon) % epsilon distance from Goal to termination
        iter = iter + 1;
        trans = f.fkine(q).t;
        x = [trans; 0; 0; 0];
        distance = sqrt(sum((posGoal - x).^2));
        
        jacobian = f.jacob0(q);
        jacobian_inv = pinv(jacobian' * jacobian) * jacobian';
        dq = velocity .* jacobian_inv * step_forward;
        q = q + dq';
        traj = [traj; q];
    end
end
