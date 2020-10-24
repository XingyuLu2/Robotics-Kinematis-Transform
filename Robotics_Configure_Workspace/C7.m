% Input: cspace -> NxN matrix: cspace(i,j)
%                  == 1 if [q_grid(i); q_grid(j)] is in collision,
%                  == 0 otherwise
% Output: padded_cspace -> NxN matrix: padded_cspace(i,j)
%                          == 1 if cspace(i,j) == 1, or some neighbor of
%                                  cell (i,j) has value 1 in cspace
%                                  (including diagonal neighbors)
%                          == 0 otherwise

function padded_cspace = C7(cspace)
    n = size(cspace, 1);
    % Expand the Resolution
    new_resolution = 200;
    q_grid = linspace(0, 2*pi, n);
    q_expand = linspace(0, 2*pi, new_resolution);
    padded_cspace = zeros(new_resolution);
    for i = 1:new_resolution
        q1 = q_expand(i);
        for j = 1:new_resolution
            q2 = q_expand(j);
            q1_old = knnsearch(q_grid', q1, 'K', 1);
            q2_old = knnsearch(q_grid', q2, 'K', 1);
            if cspace(q1_old,q2_old) == 1
                padded_cspace(i,j) = 1;
                for k = (i-1):(i+1)
                    for z = (j-1) : (j+1)
                        if (0 < k) && (k < 101) && (0 < z) && (z < 101) 
                            padded_cspace(k,z) = 1;
                        end
                    end
                end
            end
        end
    end
end