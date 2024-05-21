function [C]=LC_C_ab_Jh_CFM__Cp3FI_cv(u,Beta,Alpha,LA,RA,LE,RE,RUD,n9oV)
% LCCabJhCFMs0p2__Cp3FI_MOF_cv - Aerodynamic model for "LpC Cruise"
%
% DESCRIPTION: 
%   This script contains the aerodynamic model for the Lift+Cruise cruise regime. The script was
%   automatically generated using "GenModelCV.m" on 14-Jan-2021 18:03:02
%
% INPUTS:
%   List of column vectors containing the model explanatory variables 
%       The variables are in the following order:
%       [u,Beta,Alpha,LA,RA,LE,RE,RUD,n9oV]
%       Units: deg for angles, kts for velocity, revps/fps for n9oV
%
% OUTPUTS:
%   C - Matrix containing the model response variables in each column 
%       The variables are in the following order:
%       [CFx,CFy,CFz,CMx,CMy,CMz,CL,CD]
%       Units: nondimensional  
%
% WRITTEN BY:
%   Benjamin M. Simmons
%   Flight Dynamics Branch (D317)
%   NASA Langley Research Center
%
% REFERENCE:
%   Simmons, B. M., Buning, P. G., and Murphy, P. C., "Full-Envelope
%   Aero-Propulsive Model Identification for Lift+Cruise Aircraft Using
%   Computational Experiments," AIAA AVIATION Forum, Aug. 2021.
%   https://doi.org/10.2514/6.2021-3170
%
% HISTORY:
%   14 JAN 2021 - created and debugged, BMS
%

%
%   Explantory variable ranges used to develop model:
%             u: [     +50,    +130] kts
%          Beta: [      -6,      +6] deg
%         Alpha: [      +0,     +12] deg
%            LA: [     -30,     +30] deg
%            RA: [     -30,     +30] deg
%            LE: [     -30,     +30] deg
%            RE: [     -30,     +30] deg
%           RUD: [     -30,     +30] deg
%          n9oV: [+0.040641,+0.34554] revps/fps
%

% Conversion from natural variables to coded variables
u_cv = ( u-(9.00000000000000E+01) )/(4.00000000000000E+01);
Beta_cv = ( Beta-(-1.40359279754421E-09) )/(6.00000004283972E+00);
Alpha_cv = ( Alpha-(6.00000002274929E+00) )/(6.00000002274929E+00);
LA_cv = ( LA-(0.00000000000000E+00) )/(3.00000000000000E+01);
RA_cv = ( RA-(0.00000000000000E+00) )/(3.00000000000000E+01);
LE_cv = ( LE-(0.00000000000000E+00) )/(3.00000000000000E+01);
RE_cv = ( RE-(0.00000000000000E+00) )/(3.00000000000000E+01);
RUD_cv = ( RUD-(0.00000000000000E+00) )/(3.00000000000000E+01);
n9oV_cv = ( n9oV-(1.93090309147280E-01) )/(1.52449413151176E-01);






% CFx Model

CFx = ...
  -1.39378967520239E+00 .*              n9oV_cv  + ... 
  -7.04678493854609E-01 .*                    1  + ... 
  -5.48764062274458E-01 .*           n9oV_cv.^2  + ... 
  -1.00170248736846E-01 .*             Alpha_cv  + ... 
  +4.72828164728022E-02 .*               RUD_cv  + ... 
  +4.10667459148175E-02 .*                RE_cv  + ... 
  +4.86446902104366E-02 .*    RUD_cv .* n9oV_cv  + ... 
  -3.78922928608206E-02 .*                LE_cv  + ... 
  +4.25560224541818E-02 .*     RE_cv .* n9oV_cv  + ... 
  -3.64159369894027E-02 .*     LE_cv .* n9oV_cv  + ... 
  +1.55948538076297E-02 .*              Beta_cv  + ... 
  -1.53942717200020E-02 .*                LA_cv  + ... 
  -1.70160051033024E-02 .*    Alpha_cv .* RA_cv  + ... 
  -1.57941129201248E-02 .*    Alpha_cv .* LA_cv  + ... 
  +6.82045009252055E-02 .*           n9oV_cv.^3  + ... 
  +2.19652899722318E-02 .*             RA_cv.^2  + ... 
  -3.28270147998499E-02 .*          Alpha_cv.^2  + ... 
  +2.33219115172121E-02 .*             LA_cv.^2 ;      

% CFy Model

CFy = ...
  -8.47058528724034E-02 .*               RUD_cv  + ... 
  -5.62492951579911E-02 .*              Beta_cv  + ... 
  -5.09157981526317E-02 .*    RUD_cv .* n9oV_cv  + ... 
  -1.06072873853379E-02 .*                LA_cv  + ... 
  +1.03087722818448E-02 .*                RA_cv  + ... 
  -1.89221185723797E-02 .*   Beta_cv .* n9oV_cv  + ... 
  -1.32863906781484E-02 .*                LE_cv  + ... 
  -1.69493003018194E-02 .* RUD_cv .* n9oV_cv.^2  + ... 
  -8.42651377556217E-03 .*     LE_cv .* n9oV_cv  + ... 
  +4.85722649492783E-03 .*   Alpha_cv .* RUD_cv  + ... 
  -5.55090039508485E-03 .*      LE_cv .* RUD_cv  + ... 
  +8.03167645934651E-03 .*                RE_cv  + ... 
  -7.08550423729025E-03 .*   RE_cv .* RUD_cv.^2  + ... 
  -5.48256069249994E-03 .*      RE_cv .* RUD_cv  + ... 
  -6.84415578286183E-03 .* RE_cv .* RUD_cv .* n9oV_cv  + ... 
  +5.44049236170865E-03 .*   LE_cv .* RUD_cv.^2  + ... 
  +3.82766023502646E-03 .* Beta_cv .* Alpha_cv .* n9oV_cv  + ... 
  -4.48357638720517E-03 .* LE_cv .* RUD_cv .* n9oV_cv ;      

% CFz Model

CFz = ...
  +7.68193018615067E-01 .*                    1  + ... 
  +5.24652345627760E-01 .*             Alpha_cv  + ... 
  +3.41543612073113E-01 .*                LA_cv  + ... 
  +3.45799816195069E-01 .*                RA_cv  + ... 
  +7.89037272974986E-02 .*                LE_cv  + ... 
  +7.55130445920181E-02 .*                RE_cv  + ... 
  -3.08870856410449E-02 .*             LA_cv.^2  + ... 
  -2.25303807472791E-02 .*           Beta_cv.^2  + ... 
  +3.63520237608624E-02 .*     LE_cv .* n9oV_cv  + ... 
  -7.64591772240772E-02 .*             LA_cv.^3  + ... 
  -3.13661780249810E-02 .*             RA_cv.^2  + ... 
  -8.08557987517408E-02 .*             RA_cv.^3  + ... 
  +3.34139828429899E-02 .*     RE_cv .* n9oV_cv  + ... 
  -2.49392210873812E-02 .*          Alpha_cv.^2  + ... 
  -1.56314053249097E-02 .*    Alpha_cv .* LA_cv  + ... 
  -1.66564138946826E-02 .*    Alpha_cv .* RA_cv  + ... 
  +2.19329444231783E-02 .*  Alpha_cv .* n9oV_cv  + ... 
  +2.02585406109202E-02 .*  LE_cv.^2 .* n9oV_cv ;      

% CMx Model

CMx = ...
  +1.86237814997744E-01 .*                LA_cv  + ... 
  -1.86299389646948E-01 .*                RA_cv  + ... 
  -4.21490267717725E-02 .*                    1  + ... 
  -7.99360381150196E-02 .*              n9oV_cv  + ... 
  -1.77081049202052E-02 .*               RUD_cv  + ... 
  -3.07893251613615E-02 .*           n9oV_cv.^2  + ... 
  -4.34373572781252E-02 .*             LA_cv.^3  + ... 
  +4.34219723545197E-02 .*             RA_cv.^3  + ... 
  -9.07245893833385E-03 .*              Beta_cv  + ... 
  -1.78100956241825E-02 .*             LA_cv.^2  + ... 
  +1.70366086966858E-02 .*             RA_cv.^2  + ... 
  -7.66795949554880E-03 .*    Alpha_cv .* LA_cv  + ... 
  +7.87009335103899E-03 .*    Alpha_cv .* RA_cv  + ... 
  -8.37305563689699E-03 .*    RUD_cv .* n9oV_cv  + ... 
  -3.75363599240523E-03 .*                RE_cv ;      

% CMy Model

CMy = ...
  -1.17804662754608E+00 .*              n9oV_cv  + ... 
  -5.55189043278300E-01 .*                    1  + ... 
  -3.28343334726890E-01 .*                LE_cv  + ... 
  -2.66382907289952E-01 .*                RE_cv  + ... 
  -4.56689783728132E-01 .*           n9oV_cv.^2  + ... 
  -1.96546064648999E-01 .*     LE_cv .* n9oV_cv  + ... 
  -1.26563790948136E-01 .*     RE_cv .* n9oV_cv  + ... 
  +7.66644414342921E-02 .*               RUD_cv  + ... 
  +7.36432907555219E-02 .*    RUD_cv .* n9oV_cv  + ... 
  +7.42494045424628E-02 .*            RUD_cv.^2  + ... 
  -6.15371296900848E-02 .*  LE_cv .* n9oV_cv.^2  + ... 
  -6.23638279588042E-02 .*  Alpha_cv .* n9oV_cv  + ... 
  -3.48393012752589E-02 .*             Alpha_cv  + ... 
  -2.96658733974694E-02 .*              Beta_cv  + ... 
  -4.62943863056074E-02 .*          Alpha_cv.^2  + ... 
  -3.22481257621229E-02 .*   Beta_cv .* n9oV_cv  + ... 
  +2.91104093978117E-02 .*             LE_cv.^2  + ... 
  -4.51363583648274E-02 .*  RE_cv .* n9oV_cv.^2  + ... 
  +5.68493101540776E-02 .* RUD_cv.^2 .* n9oV_cv  + ... 
  -1.40167152319753E-02 .*                LA_cv  + ... 
  +2.44122844314686E-02 .*             RE_cv.^2 ;      

% CMz Model

CMz = ...
  +5.82955875041906E-02 .*               RUD_cv  + ... 
  +3.06869969564286E-02 .*    RUD_cv .* n9oV_cv  + ... 
  +1.30722987233709E-02 .*              Beta_cv  + ... 
  +1.40104781476535E-02 .*   Beta_cv .* n9oV_cv  + ... 
  +6.63410482518342E-03 .*                LA_cv  + ... 
  -6.80236473019296E-03 .*                RA_cv  + ... 
  +1.11853696238229E-02 .*                LE_cv  + ... 
  +6.11085006941100E-03 .*    Alpha_cv .* LA_cv  + ... 
  -5.88762347666619E-03 .*    Alpha_cv .* RA_cv  + ... 
  +8.24836404378982E-03 .*     LE_cv .* n9oV_cv  + ... 
  -1.30090718132470E-02 .*             LA_cv.^2  + ... 
  +1.25137061812583E-02 .*             RA_cv.^2  + ... 
  +7.33175256434774E-03 .* RUD_cv .* n9oV_cv.^2  + ... 
  -3.12999697180724E-03 .*   Alpha_cv .* RUD_cv  + ... 
  +3.71775050373094E-03 .*      LE_cv .* RUD_cv  + ... 
  +3.93757551255981E-03 .*      RE_cv .* RUD_cv  + ... 
  +4.69792503642570E-03 .* RE_cv .* RUD_cv .* n9oV_cv  + ... 
  -3.35612863156843E-03 .*             Alpha_cv  + ... 
  -4.08414051914268E-03 .*  Alpha_cv .* n9oV_cv  + ... 
  -3.49259619606639E-03 .*   LE_cv .* RUD_cv.^2  + ... 
  -2.97151480944143E-03 .* RUD_cv.^2 .* n9oV_cv  + ... 
  +2.96646144384295E-03 .* LE_cv .* RUD_cv .* n9oV_cv  + ... 
  -1.89390335910688E-03 .* Beta_cv .* Alpha_cv .* n9oV_cv  + ... 
  +2.98239364931891E-03 .* Beta_cv.^2 .* RUD_cv  + ... 
  -4.54652376044907E-03 .*       u_cv .* RUD_cv  + ... 
  -1.62251249105571E-03 .*           Beta_cv.^2  + ... 
  -3.27575030932028E-03 .*            RUD_cv.^3  + ... 
  -8.35110600712835E-04 .*     Beta_cv .* RE_cv  + ... 
  -1.10584112876925E-03 .* LE_cv .* RE_cv .* RUD_cv  + ... 
  +9.71136700563079E-04 .* Beta_cv .* Alpha_cv .* RE_cv  + ... 
  -5.78756804864246E-03 .* u_cv .* RUD_cv .* n9oV_cv  + ... 
  +8.35365667184321E-04 .*     Beta_cv .* LE_cv  + ... 
  +1.52823744403708E-03 .*     RE_cv .* n9oV_cv  + ... 
  +4.46067105396538E-03 .*   RE_cv .* RUD_cv.^2  + ... 
  -3.07398538221654E-03 .*             RE_cv.^3 ;      

% CL Model

CL = ...
  +8.41770568118080E-01 .*                    1  + ... 
  +5.96357643994239E-01 .*             Alpha_cv  + ... 
  +3.45344634962302E-01 .*                RA_cv  + ... 
  +3.44724268845743E-01 .*                LA_cv  + ... 
  +1.55011205475187E-01 .*              n9oV_cv  + ... 
  +1.59307025249199E-01 .*  Alpha_cv .* n9oV_cv  + ... 
  +8.22616115296399E-02 .*                LE_cv  + ... 
  +7.08530448385216E-02 .*                RE_cv  + ... 
  -3.44101178404722E-02 .*             LA_cv.^2  + ... 
  +3.98257631791045E-02 .*     LE_cv .* n9oV_cv  + ... 
  -3.37709374652752E-02 .*             RA_cv.^2  + ... 
  -7.97353458120209E-02 .*             LA_cv.^3  + ... 
  +5.36018720116333E-02 .*           n9oV_cv.^2  + ... 
  -8.07446138836924E-02 .*             RA_cv.^3  + ... 
  +2.98460036480108E-02 .*     RE_cv .* n9oV_cv  + ... 
  +5.10124355020706E-02 .* Alpha_cv .* n9oV_cv.^2  + ... 
  -2.34941969410549E-02 .*           Beta_cv.^2  + ... 
  -1.66032621788786E-02 .*    Alpha_cv .* RA_cv  + ... 
  -1.48691505939745E-02 .*    Alpha_cv .* LA_cv  + ... 
  -2.16910760910805E-02 .*          Alpha_cv.^2 ;      

% CD Model

CD = ...
  -1.36918741650927E+00 .*              n9oV_cv  + ... 
  -6.09423572518096E-01 .*                    1  + ... 
  -5.41238007687502E-01 .*           n9oV_cv.^2  + ... 
  +4.89780903950813E-02 .*                RE_cv  + ... 
  +4.58070926034607E-02 .*               RUD_cv  + ... 
  +2.98767532158525E-02 .*             Alpha_cv  + ... 
  +4.80328100112753E-02 .*    RUD_cv .* n9oV_cv  + ... 
  +4.64467236608378E-02 .*     RE_cv .* n9oV_cv  + ... 
  +1.86231068600678E-02 .*                RA_cv  + ... 
  +2.72490122650077E-02 .*          Alpha_cv.^2  + ... 
  -2.86690085513998E-02 .*                LE_cv  + ... 
  -3.28048333340855E-02 .*     LE_cv .* n9oV_cv  + ... 
  +1.63379609705416E-02 .*           Beta_cv.^3  + ... 
  +1.20756314910202E-02 .*                LA_cv  + ... 
  +2.38393474645484E-02 .*             LA_cv.^2  + ... 
  +6.49881420003747E-02 .*           n9oV_cv.^3 ;      






% Output response variable predictions
C = [CFx,CFy,CFz,CMx,CMy,CMz,CL,CD];

return