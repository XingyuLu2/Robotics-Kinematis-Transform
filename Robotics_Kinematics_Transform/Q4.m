% input: f -> a 9-joint robot encoded as a SerialLink class
%        qInit -> 1x9 vector denoting current joint configuration
%        circle -> 3xn matrix of Cartesian positions that describe the
%                  circle
%        velocity -> scalar denoting desired velocity of end effector
% output: traj -> nx9 matrix that denotes arm trajectory. Each row denotes
%                 a joint configuration. The first row is the first
%                 configuration. The last is the last configuration in the
%                 trajectory. The end effector should trace a circle in the
%                 workspace, as specified by the input argument circle.
%                 (orientation is to be ignored)

function traj = Q4(f, qInit, circle, velocity) 
    traj = [qInit];
    
    end_iter = size(circle);
    end_iter = end_iter(2);
    if end_iter < 2
        end_iter = 2;
    end
    
    for i = 2 : end_iter
        a = size(traj);
        q = traj(a(1), :);
        posGoal = circle(:, i);
        one_way = Q3(f, q, posGoal, 0.05, velocity);
        traj = [traj; one_way];
    end
end