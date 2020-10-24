% Input: q_grid -> 1xN vector of angles between 0 and 2*pi, discretizing
%                  each dimension of configuration space
%        q_start -> 2x1 vector denoting the start configuration
%        q_goal -> 2x1 vector denoting the goal configuration
%        path -> Mx2 matrix containing a collision-free path from q_start
%                to q_goal (as computed in C3, embedded in distances).
%                The entries of path are grid cell indices, i.e., integers
%                between 1 and N. The first row is the grid cell containing
%                q_start, the final row is the grid cell containing q_goal.
% Output: q_path -> Mx2 matrix containing a collision-free path from
%                   q_start to q_goal. Each row in q_path is a robot
%                   configuration. The first row should be q_start,
%                   the final row should be q_goal.

function q_path = C5(q_grid, q_start, q_goal, path)
    q_path = [q_start(1) q_start(2)];
    % for resolution = 200
    if size(path, 1) > 60
        q_grid = linspace(0, 2*pi, 200);
    end
    n = size(path);
    for i = 1:n
        x= path(i,1); y = path(i,2);
        a_x = q_grid(x);
        a_y = q_grid(y);
        q_path = [q_path; a_x a_y];
    end
    q_path = [q_path; q_goal(1) q_goal(2)];
end