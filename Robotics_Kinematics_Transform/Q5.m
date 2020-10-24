% input: f1 -> an 9-joint robot encoded as a SerialLink class for one
%              finger
%        f2 -> an 9-joint robot encoded as a SerialLink class for one
%              finger
%        qInit -> 1x11 vector denoting current joint configuration.
%                 First seven joints are the arm joints. Joints 8,9 are
%                 finger joints for f1. Joints 10,11 are finger joints
%                 for f2.
%        f1Target, f2Target -> 3x1 vectors denoting the target positions
%                              each of the two fingers.
% output: q -> 1x11 vector of joint angles that cause the fingers to
%              reach the desired positions simultaneously.
%              (orientation is to be ignored)

function q = Q5(f1, f2, qInit, f1Target, f2Target)    
    step_size = 0.1;
    error_threshold = 0.001;
    max_iter = 1000;
    
    q = qInit;
    iter = 1;
    posGoal = [f1Target; f2Target];
    while(iter < max_iter)
        iter = iter + 1;

        pos_1 = f1.fkine(q(1:9)).t;
        pos_2 = f2.fkine([q(1:7) q(10:11)]).t;
        pos = [pos_1;pos_2];

        dx = posGoal - pos;
        error = norm(dx);
        
        if (error < error_threshold)
            break
        end

        add_matrix = [0 0; 0 0; 0 0];
        jacobian_1 = f1.jacob0(q(1:9));
        jacobian_1 = jacobian_1(1:3,:);
        jacobian_1 = [jacobian_1 add_matrix];
        
        jacobian_2 = f2.jacob0([q(1:7) q(10:11)]);
        jacobian_2 = [jacobian_2(1:3, 1:7) add_matrix jacobian_2(1:3, 8:9)];
        
        jacobian = [jacobian_1; jacobian_2];

        jacobian_inv = pinv(jacobian' * jacobian) * jacobian';
        dq = step_size .* jacobian_inv * dx;
        q = q + dq';
    end
end