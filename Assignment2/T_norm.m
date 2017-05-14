function [T] = T_norm(x, y)
%T_norm Returns the T matrix needed for the normalized 8-point algorithm.
%   Detailed explanation goes here

    m_x = mean(x);
    m_y = mean(y);

    d = sum(sqrt( (x - m_x) .^ 2 + (y - m_y) .^ 2)) / size(x, 1);

    T = [sqrt(2)/d, 0, -m_x * sqrt(2)/d;
        0, sqrt(2)/d, -m_y * sqrt(2)/d;
        0, 0, 1 ];

end

