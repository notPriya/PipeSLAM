% Converts from a transformation matrix to the Rodrigues axis-angle
% formulation.
function v = HtoAxisAngle(H)
    theta = acos((trace(H(1:3, 1:3))-1)/2);
    if theta == 0
        v = [0 0 0];
        return;
    end
    v = 1/(2*sin(theta)) * [H(3, 2)-H(2, 3); H(1, 3)-H(3, 1); H(2, 1)-H(1, 2)];
    v = v'*theta;
end