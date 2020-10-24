% Input: robot -> A 4-DOF robot encoded as a SerialLink object
%        q_min -> 1x4 vector of minimum angle for each joint
%        q_max -> 1x4 vector of maximum angle for each joint
%        num_samples -> Integer denoting number of samples in PRM
%        num_neighbors -> Integer denoting number of closest neighbors to
%                         consider in PRM
%        link_radius -> Scalar denoting radius of each robot link's
%                       cylindrical body
%        sphere_centers -> Nx3 matrix containing the centers of N spherical
%                          obstacles
%        sphere_radii -> Nx1 vector containing the radii of N spherical
%                        obstacles
% Output: samples -> num_samples x 4 matrix, sampled configurations in the
%                    roadmap (vertices)
%         adjacency -> num_samples x num_samples matrix, the weighted
%                      adjacency matrix denoting edges between roadmap
%                      vertices. adjacency(i,j) == 0 if there is no edge
%                      between vertex i and j; otherwise, its value is the
%                      weight (distance) between the two vertices. For an
%                      undirected graph, the adjacency matrix should be
%                      symmetric: adjacency(i,j) == adjacency(j,i)

function [samples, adjacency] = M2(robot, q_min, q_max, num_samples, num_neighbors, link_radius, sphere_centers, sphere_radii)
    adjacency = zeros(num_samples);
    samples = [];
    
    % find all collision-free states
    count = 0;
    while true
        qs = M1(q_min, q_max, 100);
        for i = 1:100
            q = qs(i,:);
            in_collision = check_collision(robot, q, link_radius, sphere_centers, sphere_radii);
            if in_collision == false
                samples = [samples; q]; count = count + 1;
                if count == num_samples
                    break
                end
            end
        end
        if count == num_samples
            break
        end
    end
    
    % Update the adjacency by checking the collision-free points
    % and finding K nearest points
    for i = 1:num_samples
        % find the N neareast points
        q = samples(i,:);
        [near_posi, dis] = knnsearch(samples, q, 'K', num_neighbors+1);
        for j = 2:num_neighbors+1
            q_near = samples(near_posi(j),:);
            % for check whether the edge between them is in-collisition
            in_collision = check_edge(robot, q, q_near, link_radius, sphere_centers, sphere_radii);
            if in_collision == false
                % for that the adjacency should be symmetric, and for the
                % reason ">= neighbors" part in HW2 
                adjacency(i,near_posi(j)) = dis(:,j);
                adjacency(near_posi(j), i) = dis(:,j);
            end
        end
    end  
end