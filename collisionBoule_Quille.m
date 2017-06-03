function [ penetration, dir_contact ] = collisionBoule_Quille( pos_b, pos_q, dir_q )
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here
global R_b R_q H_q
%% Collision de côté :

delta_proj = (pos_b-pos_q)'*dir_q;
proj=pos_q+delta_proj*dir_q;

distance=pos_b-proj;
d = norm(distance);
dir_contact = distance/d;    %de la quille vers la boule
if(d<R_b+R_q)
    penetration = R_b + R_q - d;
else
    penetration=0;

end
%% Collision par les bases :

%TODO

end

