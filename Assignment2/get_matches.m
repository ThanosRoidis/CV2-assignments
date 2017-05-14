function [ matches ] = get_matches( im1, im2 )
%GET_MATCHES Retuns an Nx4 matrix, where N is the number of matching pairs,
% the 1st and the 2nd columnts are the (x,y) on the first image and the 3rd
% and 4rth column from the second image,  for all the matching pairs from 

    if size(im1,3) == 3
        im1 = rgb2gray(im1);
        im2 = rgb2gray(im2);
    end

    Ia = single(im1);
    Ib = single(im2);
    [fa, da] = vl_sift(Ia);
    [fb, db] = vl_sift(Ib);

    [matches_indexes, scores] = vl_ubcmatch(da, db) ;

    %append positions into one array instead of having indexes
    matches = zeros(length(matches_indexes), 4);
    for i=1:length(matches_indexes)
        matches(i,1:2) = fa(1:2,matches_indexes(1,i));
        matches(i,3:4) = fb(1:2,matches_indexes(2,i));
    end
    
    
end

