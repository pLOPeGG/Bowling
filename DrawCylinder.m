function [  ] = DrawCylinder( pos, dir )
%DrawCylinder Summary of this function goes here
%pos est la mosition du centre de la base de la quille.

global y_quille imageQuille H_q

dir=dir/norm(dir);

[phi,theta,~]=cart2sph(dir(1),dir(2),dir(3));
theta=radtodeg(-theta+pi*0.5);
phi=radtodeg(phi);

[X,Y,Z] = cylinder(y_quille);

X = X + pos(1);
Y = Y + pos(2);
Z = Z*H_q ;  % par défaut la fonction cylinder renvoie un cylindre de hauteur 1




quille = surf(X,Y,Z,'edgecolor', 'none','FaceColor','texturemap');

rotate(quille, [0,1,0], theta, pos);
rotate(quille,[0,0,1],phi,pos);
end

