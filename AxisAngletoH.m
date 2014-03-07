% Converts from a Rodrigues axis-angle formulation to a transformation
% matrix.
function H = AxisAngletoH(v)
    theta = norm(v(1:3));
    if theta == 0
        E = zeros(3, 1);
    else
        E = v(1:3)./theta;
    end
    Ex = [0 -E(3) E(2); E(3) 0 -E(1); -E(2) E(1) 0];
    R = eye(3)*cos(theta) + (1 - cos(theta))*(E*E') + Ex*sin(theta);
    H = [R v(4:6)];
end