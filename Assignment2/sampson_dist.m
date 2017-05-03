function [ d ] = sampson_dist( match, F )
%SAMPSON_DIST Summary of this function goes here
%   Detailed explanation goes here
    p = [match(0), match(1), 1];
    q = [match(2), match(3), 1];
    
    fp = F*p;
    fq = F'*q;
    
    d = (q' * F * p)^2;
    d = d/ (fp(1)^2 + fp(2)^2 + fq(1)^2 + fq(2)^2);

end

