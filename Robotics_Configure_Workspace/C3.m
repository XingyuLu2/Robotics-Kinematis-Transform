% Input: cspace -> NxN matrix: cspace(i,j)
%                   == 1 if [q_grid(i); q_grid(j)] is in collision,
%                   == 0 otherwise
%        q_grid -> 1xN vector of angles between 0 and 2*pi, discretizing
%                  each dimension of configuration space
%        q_goal -> 2x1 vector denoting the goal configuration
% Output: distances -> NxN matrix containing the distance transform from
%                      the goal configuration
%                      == 0 if cell is unreachable
%                      == 1 if cell is an obstacle
%                      == 2 if cell is the goal
%                      >  2 otherwise

function distances = C3(cspace, q_grid, q_goal)
    n = size(cspace); n = n(2);
    % for cases having other resolution values
    if n ~= 100
        q_grid = linspace(0, 2*pi, n);
    end
    distances = zeros(n);
    d_x = abs(q_grid - q_goal(1));
    d_y = abs(q_grid - q_goal(2));
    goal_x = min(d_x);
    goal_y = min(d_y);
    i_x = find(d_x==goal_x);
    i_y = find(d_y==goal_y);
    distances(i_x, i_y) = 2;
    % Draw the Obstacles
    for i = 1:n
        for j = 1:n
            if cspace(i, j) == 1
                distances(i,j) = 1;
            end
        end
    end
    que_x = [i_x]; que_y = [i_y];
    que_size = size(que_x); que_size = que_size(1);
    dis = 2;

    while que_size ~= 0
        que_update_x = [];
        que_update_y = [];
        for num = 1:que_size
            x = que_x(num,:); 
            y = que_y(num,:);
            for i = 1:8
                if i == 1
                    if 0 < y+1 && y+1 < (n+1) && 0 < x && x < (n+1)
                    pt = cspace(x,y+1);
                    if pt == 0  && distances(x, y+1) == 0
                        que_update_x = [que_update_x;x];
                        que_update_y = [que_update_y;y+1];
                        distances(x, y+1) = distances(x, y) + 1;
                    end
                    end
                end
                if i == 2
                    if 0 < y && y < (n+1) && 0 < x-1 && x-1 < (n+1)
                    pt = cspace(x-1,y);
                    if pt == 0  && distances(x-1, y) == 0
                        que_update_x = [que_update_x;x-1];
                        que_update_y = [que_update_y;y];
                        distances(x-1, y) = distances(x, y) + 1;
                    end
                    end
                end
                if i == 3
                    if 0 < y-1 && y-1 < (n+1) && 0 < x && x < (n+1)
                        pt = cspace(x,y-1);
                        if pt == 0 && distances(x, y-1) == 0
                            que_update_x = [que_update_x;x];
                            que_update_y = [que_update_y;y-1];
                            distances(x, y-1) = distances(x, y) + 1; 
                        end
                    end
                end
                if i == 4
                    if 0 < y && y < (n+1) && 0 < x+1 && x+1 < (n+1)
                    pt = cspace(x+1,y);
                    if pt == 0  && distances(x+1, y) == 0
                        que_update_x = [que_update_x;x+1];
                        que_update_y = [que_update_y;y];
                        distances(x+1, y) =  distances(x, y)+ 1; 
                    end
                    end
                end
                if i == 5
                    if 0 < y+1 && y+1 < (n+1) && 0 < x-1 && x-1 < (n+1)
                    pt = cspace(x-1,y+1);
                    if pt == 0  && distances(x-1, y+1) == 0
                        que_update_x = [que_update_x;x-1];
                        que_update_y = [que_update_y;y+1];
                        distances(x-1, y+1) = distances(x, y) + 1;
                    end
                    end
                end
                if i == 6
                    if 0 < y+1 && y+1 < (n+1) && 0 < x+1 && x+1 < (n+1)
                    pt = cspace(x+1,y+1);
                    if pt == 0  && distances(x+1, y+1) == 0
                        que_update_x = [que_update_x;x+1];
                        que_update_y = [que_update_y;y+1];
                        distances(x+1, y+1) = distances(x, y) + 1;
                    end
                    end
                end
                if i == 7
                    if 0 < y-1 && y-1 < (n+1) && 0 < x-1 && x-1 < (n+1)
                        pt = cspace(x-1,y-1);
                        if pt == 0 && distances(x, y-1) == 0
                            que_update_x = [que_update_x;x-1];
                            que_update_y = [que_update_y;y-1];
                            distances(x-1, y-1) = distances(x, y) + 1; 
                        end
                    end
                end
                if i == 8
                    if 0 < y-1 && y-1 < (n+1) && 0 < x+1 && x+1 < (n+1)
                    pt = cspace(x+1,y-1);
                    if pt == 0  && distances(x+1, y-1) == 0
                        que_update_x = [que_update_x;x+1];
                        que_update_y = [que_update_y;y-1];
                        distances(x+1, y-1) =  distances(x, y)+ 1; 
                    end
                    end
                end
            end
        end
        que_x = que_update_x;
        que_y = que_update_y;
        que_size = size(que_x); que_size = que_size(1);
        % dis = dis +1;
    end
end