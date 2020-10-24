% Input: robot -> A 4-DOF robot encoded as a SerialLink object
%        samples -> num_samples x 4 matrix, vertices in the roadmap
%        adjacency -> num_samples x num_samples matrix, the weighted
%                     adjacency matrix denoting edges in the roadmap
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

function [path, path_found] = M3(robot, samples, adjacency, q_start, q_goal, link_radius, sphere_centers, sphere_radii)
    path = []; path_found = false; % initialization for Output augments
    
    n = size(samples, 1);
    % Obtain the Graph information
    s = [];
    t = [];
    weights = [];
    for i = 1:n
        for j = 1:n
            if adjacency(i,j) ~= 0
                s = [s i];
                t = [t j];
                weights = [weights adjacency(i,j)];
            end
        end
    end
    % Find the nearest close start point
    [near_posi, dis] = knnsearch(samples, q_start, 'K', 1);
    start_point = near_posi(:,1);
    [near_posi, dis] = knnsearch(samples, q_goal, 'K', 1);
    end_point = near_posi(:,1);

    % Find the Shortest Path
    G = graph(s,t, weights);
    path_p = shortestpath(G, start_point, end_point);
    if size(path_p) > 0
        path = [path; q_start; samples(start_point,:)];
        n_p = size(path_p, 2);
        path_found = true;
        for i = 1:n_p
            d = path_p(:,i);
            path = [path; samples(d,:)];
        end
        path = [path; samples(end_point,:); q_goal];
    end
end