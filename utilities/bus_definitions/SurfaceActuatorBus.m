function cellInfo = SurfaceActuatorBus(varargin)
% SURFACEACTUATORBUS returns a cell array containing bus object information 
% 
% Optional Input: 'false' will suppress a call to Simulink.Bus.cellToObject 
%                 when the MATLAB file is executed. 
% The order of bus element attributes is as follows:
%   ElementName, Dimensions, DataType, SampleTime, Complexity, SamplingMode, DimensionsMode, Min, Max, DocUnits, Description 

suppressObject = false;
numSurf=1;
if nargin >= 1
    if islogical(varargin{1}) && varargin{1} == false 
      suppressObject = true;
    end
    if nargin == 2
        numSurf=varargin{2};
    end
end 

cellInfo = { ... 
  { ... 
    'BUS_SURF_ACTUATOR', ... 
    'BusSurfaceActuator.h', ... 
    '', ... 
    'Exported', ... 
    '-1', {... 
{'Position', numSurf, 'double', -1, 'real', 'Sample', 'Fixed', [], [], '', ''}; ...
{'Rate', numSurf, 'double', -1, 'real', 'Sample', 'Fixed', [], [], '', ''}; ...
{'Failure', 1, 'Bus: BUS_FAILURE_SURF', -1, 'real', 'Sample', 'Fixed', [], [], '', ''}; ...

    } ...
  } ...
}'; 

if ~suppressObject 
    % Create bus objects in the MATLAB base workspace 
    Simulink.Bus.cellToObject(cellInfo);
end 
