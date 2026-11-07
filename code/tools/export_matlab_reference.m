% Export fixed MATLAB reference values for Python migration comparison.
% This file does not run any robot interface or convert an eight-thruster
% result into six thrusters.
addpath(fullfile(fileparts(mfilename('fullpath')),'..','bluerov2_kprm'));
p = bluerov2Parameters();
x = [0;0;1.5;0;0;0;0;0;0;0;0;0];
f = zeros(8,1);                    % original Heavy reference only
xdot = bluerov2Dynamics(x,f,p.model);
dt = 0.01;
k1 = bluerov2Dynamics(x,f,p.model);
k2 = bluerov2Dynamics(x+dt*k1/2,f,p.model);
k3 = bluerov2Dynamics(x+dt*k2/2,f,p.model);
k4 = bluerov2Dynamics(x+dt*k3,f,p.model);
xnext = x + dt*(k1+2*k2+2*k3+k4)/6;
save(fullfile(pwd,'matlab_reference.mat'),'x','f','xdot','xnext','dt','-v7');
fprintf('Saved matlab_reference.mat (Heavy reference only, %d controls).\n',numel(f));
