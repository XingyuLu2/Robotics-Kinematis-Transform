% Input: distances -> NxN matrix containing the distance transform from
%                      the goal configuration
%                      == 0 if cell is unreachable
%                      == 1 if cell is an obstacle
%                      == 2 if cell is the goal
%                      >  2 otherwise
%        q_grid -> 1xN vector of angles between 0 and 2*pi, discretizing
%                  each dimension of configuration space
%        q_start -> 2x1 vector denoting the start configuration
% Output: path -> Mx2 matrix containing a collision-free path from q_start
%                 to q_goal (as computed in C3, embedded in distances).
%                 The entries of path should be grid cell indices, i.e.,
%                 integers between 1 and N. The first row should be the
%                 grid cell containing q_start, the final row should be
%                 the grid cell containing q_goal.

function path = C4(distances, q_grid, q_start)
    n = size(distances); n = n(2);
    % for cases having other resolution values
    if n ~= 100
        q_grid = linspace(0, 2*pi, n);
    end
    d_x = abs(q_grid - q_start(1));
    d_y = abs(q_grid - q_start(2));
    goal_x = min(d_x);
    goal_y = min(d_y);
    i_x = find(d_x==goal_x);
    i_y = find(d_y==goal_y);
    path = [i_x i_y]; i_goal = 0; j_goal = 0;
    for i = 1:n
        for j = 1:n
            if  distances(i,j) == 2
                i_goal = i;
                j_goal = j;
            end
        end
    end
    
    current = [i_x i_y]; goal = [i_goal j_goal];
    length = size(current); length = length(1);
    flg = zeros(n); find_flg = 0;
    
    while length ~= 0
        current_update = [];
        for num = 1:length   
            if current(num, :) == goal
                path = [path; current(num,:)];
                find_flg = 1;
                break
            end
            d_dis = [];
            d_posi = [];
            for i = current(num,1)-1 : current(num,1)+1
                for j = current(num,2)-1 : current(num,2) + 1
                    if 0 < i && i < n+1 && 0 < j && j < n+1 && flg(i,j) == 0
                        d = distances(i,j);
                        flg(i,j) = 1;
                        if d == 1
                            d = 9999;
                        end
                        d_dis = [d_dis d];
                        d_posi = [d_posi;i j];
                    end
                end
            end
            min_posi = find(d_dis == min(d_dis));
            min_posi = min_posi(:,1);
            mini_posi = d_posi(min_posi, :);
            path = [path; mini_posi];
            current_update = [current_update; mini_posi];
        end
        if find_flg == 1
            break
        end
        current = current_update;
        length = size(current); length= length(1);
    end
    
end