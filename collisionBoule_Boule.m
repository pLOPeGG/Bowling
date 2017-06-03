function [ penetration, dir_contact ] = collisionBoule_Boule( pos_b, r_b, pos_q, r_q )
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here
%% Collision de côté :



distance=pos_b-pos_q;
d = norm(distance);
dir_contact = distance/d;    %de q vers b
if(d<r_b+r_q)
    penetration = r_b + r_q - d;
else
    penetration=0;

end
%% Collision par les bases :

%TODO

end
