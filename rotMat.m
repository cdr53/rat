function Rout = rotMat(ang,w)
    %ang = ang*(pi/180);
    if norm(w) ~= 1
        warning('rotMat input w: norm of the input vector is not 1')
    end
    Rout = [cos(ang)+w(1)^2*(1-cos(ang)) w(1)*w(2)*(1-cos(ang))-w(3)*sin(ang) w(1)*w(3)*(1-cos(ang))+w(2)*sin(ang); ...
            w(2)*w(1)*(1-cos(ang))+w(3)*sin(ang) cos(ang)+w(2)^2*(1-cos(ang)) w(2)*w(3)*(1-cos(ang))-w(1)*sin(ang);...
            w(3)*w(1)*(1-cos(ang))-w(2)*sin(ang) w(3)*w(2)*(1-cos(ang))+w(1)*sin(ang) cos(ang)+w(3)^2*(1-cos(ang))]; 
end