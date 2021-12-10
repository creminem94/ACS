function [sys,x0,str,ts] = c(t,x,u,flag,qi, dqi)
switch flag,

  %%%%%%%%%%%%%%%%%%
  % Initialization %
  %%%%%%%%%%%%%%%%%%
  case 0,
    [sys,x0,str,ts]=mdlInitializeSizes(1, qi, dqi);

  %%%%%%%%%%%%%%%
  % Derivatives %
  %%%%%%%%%%%%%%%
  case 1,
    sys=mdlDerivatives(t,x,u);

  %%%%%%%%%%%
  % Outputs %S
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
% end csfunc

%
%=============================================================================
% mdlInitializeSizes
% Return the sizes, initial conditions, and sample times for the S-function.
%=============================================================================
%
function [sys,x0,str,ts]=mdlInitializeSizes(n, qi, dqi)

sizes = simsizes;
sizes.NumContStates  = 2*n;%q,qdot
sizes.NumDiscStates  = 0;
sizes.NumOutputs     = 2*n;
sizes.NumInputs      = n;%n dof
sizes.DirFeedthrough = 0;%no direct link to input and output (like D matrix)
sizes.NumSampleTimes = 1;

sys = simsizes(sizes);
x0  = [qi; dqi];  %starting positions and velocities
str = [];
ts  = [0 0];

% end mdlInitializeSizes
%
%=============================================================================
% mdlDerivatives
% Return the derivatives for the continuous states.
%=============================================================================
%
function sys=mdlDerivatives(t,x,u)
    N = 1;
    q = x(1:N);
    dq = x(N+1:2*N);
    tau = u;
    I = 1.5;
    F = 0.1;
    G = 0.5;
   
    ddq = (tau-F*dq-G*sin(q))/I;
    
    sys = [dq; ddq];
    

% end mdlDerivatives
%
%=============================================================================
% mdlOutputs
% Return the block outputs.
%=============================================================================
%
function sys=mdlOutputs(t,x,u)

sys = x;

% end mdlOutputs
