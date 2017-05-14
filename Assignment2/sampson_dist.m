function [ d ] = sampson_dist( match, F )
%SAMPSON_DIST Returns the Sampson Distance between two points, [match(1:2)]
%and [match(3:4)], with the fundamental matrix F.
%   Detailed explanation goes here
    p = [match(1); match(2); 1];
    q = [match(3); match(4); 1];
    
    fp = F*p;
    fq = F'*q;
    
    d = (q' * F * p)^2;
    d = d / (fp(1)^2 + fp(2)^2 + fq(1)^2 + fq(2)^2);

end

