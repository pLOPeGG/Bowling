function [ penetration, dir_contact ] = collisionQuille_Quille( pos_p, dir_p, pos_q, dir_q )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
% Version courte de calcul de collision cylindre-cylindre

global R_q H_q

%% Hypothèse : dir = axe Z constant A MODIFIER

distance = norm(pos_p-pos_q);
if(distance<2*R_q)
    penetration=2*R_q-distance;
else
    penetration=0;
end
dir_contact=(pos_q-pos_p)/distance;



end

