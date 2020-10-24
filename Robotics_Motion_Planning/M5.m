% Input: robot -> A 4-DOF robot encoded as a SerialLink object
%        path -> Nx4 matrix containing a collision-free path between
%                q_start and q_goal
%        link_radius -> Scalar denoting radius of each robot link's
%                       cylindrical body
%        sphere_centers -> Nx3 matrix containing the centers of N spherical
%                          obstacles
%        sphere_radii -> Nx1 vector containing the radii of N spherical
%                        obstacles
% Output: smoothed_path -> Nx4 matrix containing a smoothed version of the
%                          input path, where some unnecessary intermediate
%                          waypoints may have been removed

function smoothed_path = M5(robot, path, link_radius, sphere_centers, sphere_radii)
    smoothed_path = [];
    num = size(path, 1);
    disp([1 num]);
    i = 1; flg = 1;
    while i < num
        q_s = path(i, :);
        for j = flg:num
            q_t = path(j, :);
            in_collision = check_edge(robot, q_s, q_t, link_radius, sphere_centers, sphere_radii);
            if in_collision == false
                flg = j;
            else
                break;
            end
        end
        disp([i+1 flg-1]);
        % delete the angles between
        smoothed_path = [smoothed_path; path(i+1:flg-1, :)];
        i = flg;
    end
end