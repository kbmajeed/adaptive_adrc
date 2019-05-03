function [sys,x0,str,ts] =adaptive_adrc_controller(t,x,u,flag)
%quadrotorsfunc An example MATLAB file S-function for defining a continuous system.  
%   Example MATLAB file S-function implementing continuous equations: 
%      x' = f(x,u)
%      y  = h(x)
%   See sfuntmpl.m for a general S-function template.
%   See also SFUNTMPL.
%   Copyright 1990-2009 The MathWorks, Inc.
switch flag,
  %%%%%%%%%%%%%%%%%%
  % Initialization %
  %%%%%%%%%%%%%%%%%%
  case 0,
    [sys,x0,str,ts] = mdlInitializeSizes;
  %%%%%%%%%%%%%%%
  % Derivatives %
  %%%%%%%%%%%%%%%
  case 1,
    sys=mdlDerivatives(t,x,u);
  %%%%%%%%%%%
  % Outputs %
  %%%%%%%%%%%
  case 3,
    sys=mdlOutputs(t,x,u);
  %%%%%%%%%%%%%%%%%%%
  % Unhandled flags %
  %%%%%%%%%%%%%%%%%%%
  case { 2, 4, 9 },
    sys = [];
  %%%%%%%%%%%%%%%%%%%%
  % Unexpected flags %
  %%%%%%%%%%%%%%%%%%%%
  otherwise
    DAStudio.error('Simulink:blocks:unhandledFlag', num2str(flag));
end
% end quadrotorsfunc

%=============================================================================
% mdlInitializeSizes
% Return the sizes, initial conditions, and sample times for the S-function.
%=============================================================================
function [sys,x0,str,ts] = mdlInitializeSizes
sizes = simsizes;
sizes.NumContStates  = 8;
sizes.NumDiscStates  = 0;
sizes.NumOutputs     = 9;
sizes.NumInputs      = 5;
sizes.DirFeedthrough = 1;
sizes.NumSampleTimes = 1;
sys = simsizes(sizes);
x0  = [0; 0; 0; 0; 0; 0; 0; 50;];
%x0(8) = 80;
str = [];
ts  = [0 0];
% end mdlInitializeSizes

%=============================================================================
% mdlDerivatives
% Return the derivatives for the continuous states.
%=============================================================================
function sys=mdlDerivatives(~,x,u)

%% Parameters Parameters
g  = 9.81;
m  = 1; 
b  = 1;
G  = 1;
D  = 0;

%% Observer gains and parameters
%%%%% p1 = 18; p2 = 508.5; p3 = 568.5; p4 = 503.5;
% p1 = 55; p2 = 300; p3 = 996.97; p4 = 992;
%%%%%%p1 = u(5); p2 = u(6); p3 = u(7); p4 = u(8);

% plms = 100;
plms = u(5); p1 = plms; %p2 = plms; p3 = plms; p4 = plms;
p2 = 300; p3 = 996.97; p4 = 992;
%plms = u(5); p1 = plms; p2 = 2*p1; p3 = 3*p2; p4 = 4*p3;
%plms = u(5); p1 = plms; p2 = p1/2; p3 = p2/3; p4 = p3/4;

% p2 = 2*p1;
% p3 = 3*p1;
% p4 = 4*p1;

p2 = p1;

rho = 10;
%rho = 1;

%  ea = x(1) - x(4); %this is it
%  %ea = -ea;
%  eb = ea - plms*(x(2) - x(5));

ea = x(1) - x(4); %this is p12 test
eb = ea - plms*((x(2) - x(5))+(x(3) - x(6)));

% ea = x(2) - x(5); %ignore
% eb = ea - plms*(x(3)-x(6))

xdot = [  x(2) + u(4);
          x(3);
            -g - 1*D + ((G/m)*u(1));
          x(5) + p1*(x(1)-x(4));            
          x(6) + p2*(x(1)-x(4));
          x(7) + p3*(x(1)-x(4)) + b*u(1);   
                 p4*(x(1)-x(4))
               + rho*(ea)*(eb)];           
 sys = xdot;
% end mdlDerivatives

%=============================================================================
% mdlOutputs
% Return the block outputs.
%=============================================================================
function sys=mdlOutputs(t,x,u)
% Controllers Softwork
x1d     = 2; 
x1d_dot = 0;
b       = 1;
%k1 = u(2); k2 = u(3);
k1      = 4.812;    k2      = 4.1352;
u0     = k1*(x1d - x(2)) + k2*(x1d_dot - x(3)); 
%u0      = k1*(x1d - x(5)) + k2*(x1d_dot - x(6)); 
u1      = (u0 - (x(7)))/b;

p = x(8);
sys = [x(1); x(2); x(3); x(4); x(5); x(6); x(7); u1; p;];
% end mdlOutputs
