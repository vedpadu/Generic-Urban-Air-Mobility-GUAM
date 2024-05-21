function cellInfo = SensorBus(varargin) 
% SENSORBUS returns a cell array containing bus object information 
% 
% Optional Input: 'false' will suppress a call to Simulink.Bus.cellToObject 
%                 when the MATLAB file is executed. 
% The order of bus element attributes is as follows:
%   ElementName, Dimensions, DataType, SampleTime, Complexity, SamplingMode, DimensionsMode, Min, Max, DocUnits, Description 

suppressObject = false; 
if nargin == 1 && islogical(varargin{1}) && varargin{1} == false 
    suppressObject = true; 
elseif nargin > 1 
    error('Invalid input argument(s) encountered'); 
end 

cellInfo = { ... 
  { ... 
    'SensorBus', ... 
    'BusSensor.h', ... 
    '', ... 
    'Exported', ... 
    '-1', {... 
{'Omeg_BIb', 3, 'double', -1, 'real', 'Sample', 'Fixed', [], [], '', ''}; ...
{'Accel_bIb', 3, 'double', -1, 'real', 'Sample', 'Fixed', [], [], '', ''}; ...
{'Q_i2b', 4, 'double', -1, 'real', 'Sample', 'Fixed', [], [], '', ''}; ...
{'Pos_bIi', 3, 'double', -1, 'real', 'Sample', 'Fixed', [], [], '', ''}; ...
{'Vel_bIi', 3, 'double', -1, 'real', 'Sample', 'Fixed', [], [], '', ''}; ...
{'gpsLLA', 3, 'double', -1, 'real', 'Sample', 'Fixed', [], [], '', ''}; ...
{'LaserAlt', 1, 'double', -1, 'real', 'Sample', 'Fixed', [], [], '', ''}; ...
{'Euler', 1, 'Bus: EulerBus', -1, 'real', 'Sample', 'Fixed', [], [], '', ''}; ...
{'Vtot', 1, 'double', -1, 'real', 'Sample', 'Fixed', [], [], '', ''}; ...
{'gamma', 1, 'double', -1, 'real', 'Sample', 'Fixed', [], [], '', ''}; ...
{'chi', 1, 'double', -1, 'real', 'Sample', 'Fixed', [], [], '', ''}; ...
    } ...
  } ...
}'; 

if ~suppressObject 
    % Create bus objects in the MATLAB base workspace 
    Simulink.Bus.cellToObject(cellInfo) 
end 