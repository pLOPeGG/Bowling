function [ Prev_X_b, Prev_X_q ] = PrevisionPositionEulerExplicit(X_b, V_b, A_b, X_q, V_q, A_q  )
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here

global dt


Prev_X_b = X_b + dt*V_b + dt^2/2*A_b;
Prev_X_q = X_q + dt*V_q + dt^2/2*A_q;

end

