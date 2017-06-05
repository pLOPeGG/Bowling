function [ penetration, dir_contact, delta_proj ] = collisionBoule_Segment_Quille( pos_b, r_b, pos_p, pos_q)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
% Version courte de calcul de collision cylindre-cylindre

global R_bas_q R_haut_q 

if(norm(pos_b-pos_p) + norm(pos_b-pos_b) - R_bas_q - R_haut_q < norm(pos_q - pos_p))
    dir_q = (pos_q-pos_p)/norm(pos_q-pos_p);
    delta_proj = (pos_b-pos_q)'*dir_q;
    proj=pos_q+delta_proj*dir_q;

    distance=pos_b-proj;
    d = norm(distance);
    dir_contact = distance/d;    %de la quille vers la boule
    if(d<r_b)
        penetration = r_b - d;
    else
        penetration=0;

    end

else
    penetration = 0;
    dir_contact = [1 0 0];
    delta_proj =0;
end

