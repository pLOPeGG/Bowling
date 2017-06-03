function [  ] = DrawBall( pos, r)
%DrawBall Summary of this function goes here
%   Detailed explanation goes here
global  imageBoule
[x,y,z] = sphere(15);

surf(r*x+pos(1), r*y+pos(2),r*z+pos(3),'edgecolor', 'none','FaceColor','texturemap') % where (a,b,c) is center of the sphere

end

