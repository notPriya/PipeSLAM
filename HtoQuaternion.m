% Converts from a transformation matrix to a quaternion.
function v = HtoQuaternion(H)
    v = [trace(H)/4.0 ...
        (H(1,1)-H(2,2)-H(3,3)+1)/4.0 ...
        (-H(1,1)+H(2,2)-H(3,3)+1)/4.0 ...
        (-H(1,1)-H(2,2)+H(3,3)+1)/4.0];
    v(v < 0) = 0;
    v = sqrt(v);
    [~, ind] = max(v);
    if (ind == 1)
        v(2) = v(2) * sign(H(3,2)-H(2,3));
        v(3) = v(3) * sign(H(1,3)-H(3,1));
        v(4) = v(4) * sign(H(2,1)-H(1,2));
    elseif (ind == 2)
        v(1) = v(1) * sign(H(3,2)-H(2,3));
        v(3) = v(3) * sign(H(2,1)+H(1,2));
        v(4) = v(4) * sign(H(1,3)+H(3,1));
    elseif (ind == 3)
        v(1) = v(1) * sign(H(1,3)-H(3,1));
        v(2) = v(2) * sign(H(2,1)+H(1,2));
        v(4) = v(4) * sign(H(3,2)+H(2,3));
    elseif (ind == 4)
        v(1) = v(1) * sign(H(2,1)-H(1,2));
        v(2) = v(2) * sign(H(1,3)+H(3,1));
        v(3) = v(3) * sign(H(3,2)+H(2,3));
    end
end