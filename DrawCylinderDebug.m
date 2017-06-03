function [  ] = DrawCylinderDebug( pos1, pos2 )
%DrawCylinder Summary of this function goes here
%pos est la mosition du centre de la base de la quille.

global R_bas_q R_haut_q

DrawBall(pos1, R_bas_q);
DrawBall(pos2,R_haut_q);
end

