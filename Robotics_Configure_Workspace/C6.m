% Input: robot -> A 2-DOF robot encapsulated in a MATLAB cell with fields:
%                 robot.link1, robot.link2, robot.pivot1, robot.pivot2
%                 See description in hw2_cspace and demo code in C1.
%        obstacles -> 1xN vector of polyshape objects describing N 2-D
%                     polygonal obstacles
%        q_path -> Mx2 matrix containing a collision-free path from
%                  q_start to q_goal. Each row in q_path is a robot
%                  configuration. The first row should be q_start,
%                  the final row should be q_goal.
% Output: num_collisions -> Number of swept-volume collisions encountered
%                           between consecutive configurations in q_path

function num_collisions = C6(robot, obstacles, q_path)
    num_collisions = 0;
    n = size(q_path);
    for i = 1:n-1
        q = [q_path(i,1); q_path(i,2)];
        [poly1_1, poly2_1, pivot1_1, pivot2_1] = q2poly(robot, q);
        q = [q_path(i+1,1); q_path(i+1,2)];
        [poly1_2, poly2_2, pivot1_2, pivot2_2] = q2poly(robot, q);
        
        link_1 = [poly1_1.Vertices;poly1_2.Vertices];
        link_2 = [poly2_1.Vertices;poly2_2.Vertices];
        conv1 = convhull(polyshape(link_1(:,1), link_1(:,2)));
        conv2 = convhull(polyshape(link_2(:,1), link_2(:,2)));
        
        colli_1 = intersect(obstacles, conv1); 
        colli_2 = intersect(obstacles, conv2);
        check = 0;
        for z = 1:5
            check = check + colli_1(z).NumRegions + colli_2(z).NumRegions;
        end
        if check > 0
            % the conv hulls computed
            plot(conv1);
            plot(conv2);
            % the collision conditions detected
            C1(robot, [q_path(i,1); q_path(i,2)]);
            C1(robot, [q_path(i+1,1); q_path(i+1,2)]);
            num_collisions = num_collisions + 1;
        end
    end
end