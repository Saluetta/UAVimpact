function ManipulatorDynamicsSFunction(block)
setup(block);

function setup(block)

  block.NumInputPorts  = 14;
  block.NumOutputPorts = 6;
    
  % Set up the port properties to be inherited or dynamic.
    for i = 1:14; % These are the UAV states
  block.InputPort(i).Dimensions        = 1;
  block.InputPort(i).DirectFeedthrough = false;
  block.InputPort(i).SamplingMode      = 'Sample';
  end
   %------
  for i = 1:6; % These are  x-y-z-phi-theta-psi end-effector outputs
  block.OutputPort(i).Dimensions       = 1;
  block.OutputPort(i).SamplingMode     = 'Sample';
  end

  % Register the parameters to be inputted.
  block.NumDialogPrms     = 0;
  
  % Set up the continuous states.
  block.NumContStates = 6;
  block.SampleTimes = [0 0];
  
  % -----------------------------------------------------------------
  % Options
  % -----------------------------------------------------------------
  block.SetAccelRunOnTLC(false);
  block.SimStateCompliance = 'DefaultSimState';
  block.RegBlockMethod('InitializeConditions', @InitializeConditions);
  block.RegBlockMethod('Outputs', @Outputs);
  
function InitializeConditions(block)
% Initialize 6 output States






init = [x_e,y_e,z_e,pitch_e, roll_e, yaw_e];

for i=1:6
block.OutputPort(i).Data = init(i);
block.ContStates.Data(i) = init(i);
end

function Outputs(block)
for i = 1:6;
  block.OutputPort(i).Data = block.ContStates.Data(i);
end
