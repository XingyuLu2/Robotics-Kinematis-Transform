% Input: robot -> A 2-DOF robot encapsulated in a MATLAB cell with fields:
%                 robot.link1, robot.link2, robot.pivot1, robot.pivot2
%                 See description in hw2_cspace and demo code in C1.
%        obstacles -> 1xN vector of polyshape objects describing N 2-D
%                     polygonal obstacles
%        q_grid -> 1xN vector of angles between 0 and 2*pi, discretizing
%                  each dimension of configuration space
% Output: cspace -> NxN matrix: cspace(i,j)
%                   == 1 if [q_grid(i); q_grid(j)] is in collision,
%                   == 0 otherwise

function cspace = C2(robot, obstacles, q_grid)
  num = size(q_grid);
  num = num(2);
  cspace = zeros(num);
  for i = 1 : num
      q1 = q_grid(i);
      for j = 1 : num
          q2 = q_grid(j);
          % Translate frame origins
          trans1 = [cos(q1) -sin(q1); sin(q1) cos(q1)];
          trans2 = [cos(q2) -sin(q2); sin(q2) cos(q2)];
          origin1 = robot.pivot1;
          origin2 = origin1 + trans1 * robot.pivot2;
          % Compute link polygon corners
          link1 = trans1 * robot.link1 + origin1;
          link2 = trans1 * trans2 * robot.link2 + origin2;

          r1 = polyshape(link1(1,:), link1(2,:));
          r2 = polyshape(link2(1,:), link2(2,:));

          colli_1 = intersect(obstacles, r1); 
          colli_2 = intersect(obstacles, r2);
          check = 0;
          for z = 1:5
              check = check + colli_1(z).NumRegions + colli_2(z).NumRegions;
          end
          if check > 0
              cspace(i,j) = 1;
          end
      end
  end
end