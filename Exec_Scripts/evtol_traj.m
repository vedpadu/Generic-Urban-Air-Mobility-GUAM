%% sim parameters
model = 'GUAM';
% use timeseries input
userStruct.variants.refInputType=3; % 1=FOUR_RAMP, 2= ONE_RAMP, 3=Timeseries, 4=Piecewise Bezier, 5=Default(doublets)
userStruct.variants.fmType      = 1; % 1 = Aerodynamic model, 1 = s-function, 2 = polynomial
%userStruct.variants.vehicleType = 1;

%% setup trajectory and pass to target
%time        = [0 10:1:40]'; % Column vector of time points
%time = [0 10 12 22 25 35]';
%time = [0 10 13 14 23 26 27 36 39]';
time = [linspace(0, 30, 100) linspace(31, 61, 100) linspace(62, 92, 100)]';

N_time = length(time);

vel     = zeros(N_time, 3);
vel_i   = zeros(N_time, 3);
pos     = zeros(N_time, 3);
chi     = zeros(N_time, 3);
chid    = zeros(N_time, 3);

% define trajectory as climbing right hand turn for 90 deg
% prescibe inertial position (NED)
%pos = zeros([N_time 3]);
%pos = [0 0 0; 0 0 -100; 0 0 -100; 200 0 -100; 200 0 -100; 200 0 0];

[pos(1:100, :), vel_i(1:100, :)] = generate_Motion_Profile([0, 0, -100], [0, 0, 0], 0, 30, 100);
[pos(101:200, :), vel_i(101:200, :)] = generate_Motion_Profile([200, 0, -100], [0, 0, -100], 0, 30, 100);
[pos(201:300, :), vel_i(201:300, :)] = generate_Motion_Profile([200, 0, 0], [200, 0, -100], 0, 30, 100);


%vel_i(:,1)  = gradient(pos(:,1))./gradient(time); 
%vel_i(:,2)  = gradient(pos(:,2))./gradient(time); 
%vel_i(:,3)  = gradient(pos(:,3))./gradient(time); 
%pos = [0 0 0; 0 0 -100; 0 0 -100; 0 0 -100; 200 0 -100; 200 0 -100; 200 0 -100; 200 0 0; 200 0 0];


%vel_i = [0 0 -10; 0 0 -4; 0 0 0; 20 0 0; 7 0 0; 0 0 0; 0 0 10; 0 0 4; 0 0 0];

% Compute heading
chi     = atan2(vel_i(:,2),vel_i(:,1));
chid    = gradient(chi)./gradient(time);

% add stars library blocks for quaternion functions
addpath(genpath('lib'));

% compute velocity in heading frame
q = QrotZ(chi);
vel = Qtrans(q,vel_i);

% setup trajectory to match bus
RefInput.Vel_bIc_des    = timeseries(vel,time); % Heading frame velocity
RefInput.pos_des        = timeseries(pos,time); % Inertial Position
RefInput.chi_des        = timeseries(chi,time); % Heading Angle
RefInput.chi_dot_des    = timeseries(chid,time); % Heading Angle Rate
RefInput.vel_des        = timeseries(vel_i,time); % Inertial Velocity

%target = struct('tas', 110, 'gndtrack', 0, 'stopTime', 10);
target.RefInput = RefInput;

%% Prepare to run simulation
% set initial conditions and add trajectory to SimInput
simSetup;
open(model);
