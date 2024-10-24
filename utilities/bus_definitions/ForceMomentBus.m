function cellInfo = ForceMomentBus(varargin)
% FORCEMOMENTBUS returns a cell array containing bus object information 
% 
% Optional Input: 'false' will suppress a call to Simulink.Bus.cellToObject 
%                 when the MATLAB file is executed. 
% The order of bus element attributes is as follows:
%   ElementName, Dimensions, DataType, SampleTime, Complexity, SamplingMode, DimensionsMode, Min, Max, DocUnits, Description 

suppressObject = false;
numEng=1;
if nargin >= 1
    if islogical(varargin{1}) && varargin{1} == false 
      suppressObject = true;
    end
    if nargin == 2
        numEng=varargin{2};
    end
end 

cellInfo = { ... 
  { ... 
    'ForceMomentBus', ... 
    'BusForceMoment.h', ... 
    '', ... 
    'Exported', ... 
    '-1', {... 
{'Total', 1, 'Bus: TotalFMBus', -1, 'real', 'Sample', 'Fixed', [], [], '', ''}; ...
{'Aerodynamics', 1, 'Bus: AeroFMBus', -1, 'real', 'Sample', 'Fixed', [], [], '', ''}; ...
{'Propulsion', 1, 'Bus: EngineFMBus', -1, 'real', 'Sample', 'Fixed', [], [], '', ''}; ...
    } ...
  } ...
  { ... 
    'TotalFMBus', ... 
    'BusTotalFM.h', ... 
    '', ... 
    'Exported', ... 
    '-1', {... 
{'Force_b', 3, 'double', -1, 'real', 'Sample', 'Fixed', [], [], '', ''}; ...
{'Moment_b', 3, 'double', -1, 'real', 'Sample', 'Fixed', [], [], '', ''}; ...
{'H_b', 3, 'double', -1, 'real', 'Sample', 'Fixed', [], [], '', ''}; ...
{'Hdot_b', 3, 'double', -1, 'real', 'Sample', 'Fixed', [], [], '', ''}; ...
    } ...
  } ...
  { ... 
    'AeroFMBus', ... 
    'BusAeroFM.h', ... 
    '', ... 
    'Exported', ... 
    '-1', {... 
{'Force_b', 3, 'double', -1, 'real', 'Sample', 'Fixed', [], [], '', ''}; ...
{'Moment_b', 3, 'double', -1, 'real', 'Sample', 'Fixed', [], [], '', ''}; ...
    } ...
  } ...
  { ... 
    'EngineFMBus', ... 
    'BusEngineFM.h', ... 
    '', ... 
    'Exported', ... 
    '-1', {... 
{'Force_b', 3, 'double', -1, 'real', 'Sample', 'Fixed', [], [], '', ''}; ...
{'Moment_b', 3, 'double', -1, 'real', 'Sample', 'Fixed', [], [], '', ''}; ...
{'H_b', 3, 'double', -1, 'real', 'Sample', 'Fixed', [], [], '', ''}; ...
{'Hdot_b', 3, 'double', -1, 'real', 'Sample', 'Fixed', [], [], '', ''}; ...
{'Fprop_r', numEng, 'double', -1, 'real', 'Sample', 'Fixed', [], [], '', ''}; ...
{'Mprop_r', numEng, 'double', -1, 'real', 'Sample', 'Fixed', [], [], '', ''}; ...
{'Mmotor_r', numEng, 'double', -1, 'real', 'Sample', 'Fixed', [], [], '', ''}; ...
{'Hdot_r', numEng, 'double', -1, 'real', 'Sample', 'Fixed', [], [], '', ''}; ...
{'H_r', numEng, 'double', -1, 'real', 'Sample', 'Fixed', [], [], '', ''}; ...
{'Omega_r', numEng, 'double', -1, 'real', 'Sample', 'Fixed', [], [], '', ''}; ...
{'AdvanceRatio', numEng, 'double', -1, 'real', 'Sample', 'Fixed', [], [], '', ''}; ...
    } ...
  } ...
}'; 

if ~suppressObject 
    % Create bus objects in the MATLAB base workspace 
    Simulink.Bus.cellToObject(cellInfo);
end 
%FORCEMOMENTBUS Summary of this function goes here
