% Input: robot -> A 4-DOF robot encoded as a SerialLink object
%        q_min -> 1x4 vector of minimum angle for each joint
%        q_max -> 1x4 vector of maximum angle for each joint
%        q_start -> 1x4 vector denoting the start configuration
%        q_goal -> 1x4 vector denoting the goal configuration
%        link_radius -> Scalar denoting radius of each robot link's
%                       cylindrical body
%        sphere_centers -> Nx3 matrix containing the centers of N spherical
%                          obstacles
%        sphere_radii -> Nx1 vector containing the radii of N spherical
%                        obstacles
% Output: path -> Nx4 matrix containing a collision-free path between
%                 q_start and q_goal, if a path is found. The first row
%                 should be q_start, the final row should be q_goal.
%         path_found -> Boolean denoting whether a path was found

function [path, path_found] = M4(robot, q_min, q_max, q_start, q_goal, link_radius, sphere_centers, sphere_radii)
    % For Initialization of Output Augments
    path = [];
    path_found = false;
    num_nodes = 2500; alpha = 0.1; beta = 0.1;
    
    % RRT Implementation
    % path = [path  q_start];
    V = [q_start]; E = [];
    for i = 1:num_nodes
        prob_goal = rand();
        % for select ramdom point in Q
        qs = M1(q_min, q_max, 100);
        if prob_goal < beta
            q_target = q_goal;
        else
            index = datasample(1:100, 1, 'Replace', false);
            q_target = qs(index,:); 
        end
        % find nearest point in V
        [q_near_idx, dis] = knnsearch(V, q_target, 'K', 1);
        idx = q_near_idx(:,1);
        % if V(idx,:) == q_target
        %     idx = q_near_idx(:,2);
        % end
        q_near = V(idx,:);
        q_new = q_near + (alpha / dis(:,1)).*(q_target - q_near);
        in_collision = check_collision(robot, q_new, link_radius, sphere_centers, sphere_radii);
        if in_collision == false
            V = [V; q_new];
            E = [E; idx size(V, 1)];
        end
    end
    
    % Convert the tree into a graph-type data
    n = size(E, 1);
    s = []; t = [];
    for i = 1:n
        near_idx = E(i,1);
        new_idx = E(i,2);
        q_near = V(near_idx,:);
        q_new = V(new_idx, :);
        in_collision = check_edge(robot, q_near, q_new, link_radius, sphere_centers, sphere_radii);
        if in_collision == false
            s = [s near_idx];
            t = [t new_idx];
        end
    end
    
    G = graph(s,t);
    [near_posi, dis] = knnsearch(V, q_goal, 'K', 1);
    end_point = near_posi(:,1);
    disp(end_point);
    path_p = shortestpath(G, 1, end_point);
    if size(path_p) > 0
        n_p = size(path_p, 2);
        path_found = true;
        for i = 1:n_p
            d = path_p(:,i);
            path = [path; V(d,:)];
        end
        path = [path; q_goal];
    end
end