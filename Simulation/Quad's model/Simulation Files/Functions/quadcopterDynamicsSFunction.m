function quadcopterDynamicsSFunction(block)
setup(block);

function setup(block)

  % Register the number of ports.
  block.NumInputPorts  = 5;
  block.NumOutputPorts = 12;
  
  % Set up the port properties to be inherited or dynamic.
  
  for i = 1:4; % These are the motor inputs
  block.InputPort(i).Dimensions        = 1;
  block.InputPort(i).DirectFeedthrough = false;
  block.InputPort(i).SamplingMode      = 'Sample';
  end
  %------
  % This is the disturbance input
  block.InputPort(5).Dimensions        = 6; % torques x,y,z; forces x,y,z.
  block.InputPort(5).DirectFeedthrough = false;
  block.InputPort(5).SamplingMode      = 'Sample';
  %------
  for i = 1:12;
  block.OutputPort(i).Dimensions       = 1;
  block.OutputPort(i).SamplingMode     = 'Sample';
  end

  % Register the parameters.
  block.NumDialogPrms     = 2;
  
  % Set up the continuous states.
  block.NumContStates = 12;

  % Register the sample times.
  %  [0 offset]            : Continuous sample time
  %  [positive_num offset] : Discrete sample time
  %
  %  [-1, 0]               : Inherited sample time
  %  [-2, 0]               : Variable sample time
  block.SampleTimes = [0 0];
  
  % -----------------------------------------------------------------
  % Options
  % -----------------------------------------------------------------
  % Specify if Accelerator should use TLC or call back to the 
  % MATLAB file
  block.SetAccelRunOnTLC(false);
  block.SimStateCompliance = 'DefaultSimState';
  block.RegBlockMethod('CheckParameters', @CheckPrms);
  block.RegBlockMethod('InitializeConditions', @InitializeConditions);
  block.RegBlockMethod('Outputs', @Outputs);
  block.RegBlockMethod('Derivatives', @Derivatives);
  

 function CheckPrms(block)
     quad   = block.DialogPrm(1).Data;
     IC     = block.DialogPrm(2).Data;

function InitializeConditions(block)
% Initialize 12 States

IC = block.DialogPrm(2).Data;

% IC.P, IC.Q, IC.R are in deg/s ... convert to rad/s
P = IC.P*pi/180; Q = IC.Q*pi/180; R = IC.R*pi/180; 
% IC.Phi, IC.The, IC.Psi are in deg ... convert to rads
Phi = IC.Phi*pi/180; The = IC.The*pi/180; Psi = IC.Psi*pi/180;
U = IC.U; V = IC.V; W = IC.W; 
X = IC.X; Y = IC.Y; Z = IC.Z;

init = [P,Q,R,Phi,The,Psi,U,V,W,X,Y,Z];
for i=1:12
block.OutputPort(i).Data = init(i);
block.ContStates.Data(i) = init(i);
end

function Outputs(block)
for i = 1:12;
  block.OutputPort(i).Data = block.ContStates.Data(i);
end

function Derivatives(block)
% Name all the states and motor inputs

% Load model data selected in parameter block
%which('model')

quad = block.DialogPrm(1).Data;

% P Q R in units of rad/sec
P = block.ContStates.Data(1);
Q = block.ContStates.Data(2);
R = block.ContStates.Data(3);
% Phi The Psi in radians
Phi = block.ContStates.Data(4);
The = block.ContStates.Data(5);
Psi = block.ContStates.Data(6);
% U V W in units of m/s
U = block.ContStates.Data(7);
V = block.ContStates.Data(8);
W = block.ContStates.Data(9);
% X Y Z in units of m
X = block.ContStates.Data(10);
Y = block.ContStates.Data(11);
Z = block.ContStates.Data(12);
% w values in rev/min! NOT radians/s!!!!
w1 = block.InputPort(1).Data;
w2 = block.InputPort(2).Data;
w3 = block.InputPort(3).Data;
w4 = block.InputPort(4).Data;
w  = [w1; w2; w3; w4];

%------
Dist_tau = block.InputPort(5).Data(1:3);
Dist_F   = block.InputPort(5).Data(4:6);
%------

% CALCULATE MOMENT AND THRUST FORCES
% Total Moment due to motor speeds
% Moment should be in units of N*m
% The experimental determination of Ct and Cq should be adjusted to
% model using kg instead of ounces or lb
% Mb = (quad.dctcq*(w.^2)) + (Dist_tau);  %(dctcq*(w.^2)); % Mb = [tau1 tau2 tau3]'
 tau_motorGyro = [Q*quad.Jm*2*pi/60*(-w1-w3+w2+w4); P*quad.Jm*2*pi/60*(w1+w3-w2-w4); 0]; % Note: 2*pi/60 required to convert from RPM to radians/s
 Mb = (quad.dctcq*(w.^2))+ tau_motorGyro + (Dist_tau);  % Mb = [tau1 tau2 tau3]'

% Thrust due to motor speed
% Force should be in units of Newtons for simplicity in calculating
% the acceleration in the angular velocity state equation
Fb = [0; 0; sum(quad.ct*(w.^2))];   %[0, 0, sum(ct*w.^2)]'

% Obtain dP dQ dR
omb_bi = [P; Q; R];
OMb_bi = [ 0,-R, Q;
           R, 0,-P;
          -Q, P, 0];

b_omdotb_bi = quad.Jbinv*(Mb-OMb_bi*quad.Jb*omb_bi);
H_Phi = [1,tan(The)*sin(Phi), tan(The)*cos(Phi);
         0,         cos(Phi),         -sin(Phi);
         0,sin(Phi)/cos(The),cos(Phi)/cos(The)];   
Phidot = H_Phi*omb_bi;

% Compute Rotation Matrix
% We use a Z-Y-X rotation
Rib = [cos(Psi)*cos(The) cos(Psi)*sin(The)*sin(Phi)-sin(Psi)*cos(Phi) cos(Psi)*sin(The)*cos(Phi)+sin(Psi)*sin(Phi);
       sin(Psi)*cos(The) sin(Psi)*sin(The)*sin(Phi)+cos(Psi)*cos(Phi) sin(Psi)*sin(The)*cos(Phi)-cos(Psi)*sin(Phi);
       -sin(The)         cos(The)*sin(Phi)                            cos(The)*cos(Phi)];
Rbi = Rib';
ge = [0; 0; -quad.g];
gb = Rbi*ge;
Dist_Fb = Rbi*Dist_F;

% Compute Velocity and Position derivatives of body frame
vb = [U;V;W];
b_dv = (1/quad.mass)*Fb+gb+Dist_Fb-OMb_bi*vb; % Acceleration in body frame (FOR VELOCITY)
i_dp = Rib*vb; % Units OK SI: Velocity of body frame w.r.t inertia frame (FOR POSITION)

dP = b_omdotb_bi(1);
dQ = b_omdotb_bi(2);
dR = b_omdotb_bi(3);
dPhi = Phidot(1);
dTheta = Phidot(2);
dPsi = Phidot(3);
dU = b_dv(1);
dV = b_dv(2);
dW = b_dv(3);
dX = i_dp(1);
dY = i_dp(2);
dZ = i_dp(3);
% Rough rule to impose a "ground" boundary...could easily be improved...
if ((Z<=0) && (dZ<=0)) % better  version then before?
    dZ = 0;
    block.ContStates.Data(12) = 0;
end
f = [dP dQ dR dPhi dTheta dPsi dU dV dW dX dY dZ].';
  %This is the state derivative vector
block.Derivatives.Data = f;