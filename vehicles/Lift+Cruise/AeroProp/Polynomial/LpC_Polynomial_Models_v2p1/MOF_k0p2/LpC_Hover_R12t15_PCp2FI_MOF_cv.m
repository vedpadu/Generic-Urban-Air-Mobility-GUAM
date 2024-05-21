function [C]=LpC_Hover_R12t15_PCp2FI_MOF_cv(u,v,w,LA,RA,LE,RE,RUD,N1,N2,N3,N4,N5,N6,N7,N8)
% LpC_Hover_R12t15_PCp2FI_MOF_cv - Aerodynamic model for "LpC Hover"
%
% DESCRIPTION: 
%   This script contains the aerodynamic model for the Lift+Cruise hover regime. The script was
%   automatically generated using "GenModelCV.m" on 16-Mar-2021 14:08:05
%
% INPUTS:
%   List of column vectors containing the model explanatory variables 
%       The variables are in the following order:
%       [u,v,w,LA,RA,LE,RE,RUD,N1,N2,N3,N4,N5,N6,N7,N8]
%       Units: deg for angles, kts for velocity, rpm for rotor speed
%
% OUTPUTS:
%   C - Matrix containing the model response variables in each column 
%       The variables are in the following order:
%       [Faxial,Fside,Fnormal,Mroll,Mpitch,Myaw,Flift,Fdrag]
%       Units: lbf for forces, ft-lbf for moments
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
%   16 MAR 2021 - created and debugged, BMS
%

%
%   Explantory variable ranges used to develop model:
%             u: [     +20,     +45] kts
%             v: [     -10,     +10] kts
%             w: [     -10,     +10] kts
%            LA: [     -30,     +30] deg
%            RA: [     -30,     +30] deg
%            LE: [     -30,     +30] deg
%            RE: [     -30,     +30] deg
%           RUD: [     -30,     +30] deg
%            N1: [    +550,   +1550] rpm
%            N2: [    +550,   +1550] rpm
%            N3: [    +550,   +1550] rpm
%            N4: [    +550,   +1550] rpm
%            N5: [    +550,   +1550] rpm
%            N6: [    +550,   +1550] rpm
%            N7: [    +550,   +1550] rpm
%            N8: [    +550,   +1550] rpm
%

% Conversion from natural variables to coded variables
u_cv = ( u-(3.25000000000000E+01) )/(1.25000000000000E+01);
v_cv = ( v-(0.00000000000000E+00) )/(1.00000000000000E+01);
w_cv = ( w-(0.00000000000000E+00) )/(1.00000000000000E+01);
LA_cv = ( LA-(0.00000000000000E+00) )/(3.00000000000000E+01);
RA_cv = ( RA-(0.00000000000000E+00) )/(3.00000000000000E+01);
LE_cv = ( LE-(0.00000000000000E+00) )/(3.00000000000000E+01);
RE_cv = ( RE-(0.00000000000000E+00) )/(3.00000000000000E+01);
RUD_cv = ( RUD-(0.00000000000000E+00) )/(3.00000000000000E+01);
N1_cv = ( N1-(1.05000000000000E+03) )/(5.00000000000000E+02);
N2_cv = ( N2-(1.05000000000000E+03) )/(5.00000000000000E+02);
N3_cv = ( N3-(1.05000000000000E+03) )/(5.00000000000000E+02);
N4_cv = ( N4-(1.05000000000000E+03) )/(5.00000000000000E+02);
N5_cv = ( N5-(1.05000000000000E+03) )/(5.00000000000000E+02);
N6_cv = ( N6-(1.05000000000000E+03) )/(5.00000000000000E+02);
N7_cv = ( N7-(1.05000000000000E+03) )/(5.00000000000000E+02);
N8_cv = ( N8-(1.05000000000000E+03) )/(5.00000000000000E+02);






% Faxial Model

Faxial = ...
  +3.57163464004787E+02 .*                    1  + ... 
  +1.72250460095601E+02 .*                 u_cv  + ... 
  -1.00664019975519E+02 .*                 w_cv  + ... 
  +4.25375954503909E+01 .*                N2_cv  + ... 
  +3.96475406620995E+01 .*                N8_cv  + ... 
  +3.73740150587072E+01 .*                N1_cv  + ... 
  +3.57022119428262E+01 .*                N5_cv  + ... 
  +3.57652713241289E+01 .*                N3_cv  + ... 
  +1.95799086641126E+01 .*             N7_cv.^3  + ... 
  +2.75950819312312E+01 .*                N4_cv  + ... 
  +2.34179298534522E+01 .*            RUD_cv.^2  + ... 
  +2.64344029570394E+01 .*             N6_cv.^3  + ... 
  -2.29024305272203E+01 .*        v_cv .* N4_cv  + ... 
  -2.14828021886866E+01 .*         u_cv .* w_cv  + ... 
  +2.20298889600397E+01 .*        v_cv .* N6_cv  + ... 
  +2.36570239325772E+01 .*        u_cv .* N3_cv  + ... 
  -2.00011518104372E+01 .*        w_cv .* N4_cv  + ... 
  +2.21533223804505E+01 .*        u_cv .* N5_cv  + ... 
  -2.07006074481989E+01 .*        w_cv .* N6_cv  + ... 
  +2.16273209569521E+01 .*              u_cv.^2  + ... 
  -4.88761204323910E+01 .*              w_cv.^2  + ... 
  +1.53201492652462E+01 .*        w_cv .* N3_cv  + ... 
  +1.52988974723337E+01 .*        v_cv .* N3_cv  + ... 
  -1.35673489569457E+01 .*        v_cv .* N1_cv  + ... 
  +1.83861054903363E+01 .*             N4_cv.^2  + ... 
  -1.21759114476115E+01 .*        v_cv .* N5_cv  + ... 
  -1.02005324451959E+01 .*        w_cv .* N7_cv  + ... 
  +1.33610162541634E+01 .*        w_cv .* N5_cv  + ... 
  +1.04711092983707E+01 .*       N3_cv .* N4_cv  + ... 
  +1.22048537597243E+01 .*        v_cv .* N7_cv  + ... 
  +2.41786689217314E+01 .*                LA_cv  + ... 
  -9.59318910204651E+00 .*       N4_cv .* N6_cv  + ... 
  +8.25938344606145E+00 .*                RA_cv  + ... 
  +8.99450741810891E+00 .*        u_cv .* N1_cv  + ... 
  +1.23344577377469E+01 .*             RA_cv.^2  + ... 
  +9.84934788799272E+00 .*        u_cv .* N8_cv  + ... 
  -9.05332116284450E+00 .*       N2_cv .* N6_cv  + ... 
  -9.46284713091043E+00 .*       LA_cv .* RA_cv  + ... 
  +8.53390017237989E+00 .*       LA_cv .* N4_cv  + ... 
  +8.84294253416807E+00 .*        u_cv .* N7_cv  + ... 
  -8.48698046382440E+00 .*       LA_cv .* LE_cv  + ... 
  +9.02843623774820E+00 .*       N2_cv .* N3_cv  + ... 
  -7.88123817733996E+00 .*       LE_cv .* N1_cv  + ... 
  +9.11892047738407E+00 .*       N7_cv .* N8_cv  + ... 
  +8.64064400542505E+00 .*       N2_cv .* N5_cv  + ... 
  +8.63490294749971E+00 .*       N5_cv .* N8_cv  + ... 
  +1.64480232382143E+01 .*             N2_cv.^2  + ... 
  -7.44932455058234E+00 .*       N1_cv .* N4_cv  + ... 
  -7.37610422064563E+00 .*       N6_cv .* N7_cv  + ... 
  +7.26047525584000E+00 .*      LA_cv .* RUD_cv  + ... 
  +7.32561717794392E+00 .*       N1_cv .* N5_cv  + ... 
  +1.44480899008964E+01 .*             N6_cv.^2  + ... 
  -5.85835270229044E+00 .*      RE_cv .* RUD_cv  + ... 
  -5.97909202749490E+00 .*       N1_cv .* N8_cv  + ... 
  -6.52168652154876E+00 .*       RE_cv .* N4_cv  + ... 
  +6.65052019975806E+00 .*       RA_cv .* N5_cv  + ... 
  +5.88582880958459E+00 .*        u_cv .* LA_cv  + ... 
  -5.95287683765903E+00 .*        u_cv .* LE_cv  + ... 
  -5.91194020669595E+00 .*      RUD_cv .* N8_cv  + ... 
  +1.66567637352862E+01 .*                N7_cv  + ... 
  -5.33566151429588E+00 .*       RA_cv .* N3_cv  + ... 
  -5.21309229002035E+00 .*       LE_cv .* N7_cv  + ... 
  -1.60053109200444E+01 .*             LA_cv.^3 ;      

% Fside Model

Fside = ...
  -2.18405114064757E+02 .*                N3_cv  + ... 
  +1.90373503721396E+02 .*                N5_cv  + ... 
  -1.82699631885286E+02 .*                 v_cv  + ... 
  -1.60597565229015E+02 .*                N4_cv  + ... 
  +1.53675038697068E+02 .*                N6_cv  + ... 
  -4.97229913699655E+01 .*               RUD_cv  + ... 
  +3.62437584415461E+01 .*         u_cv .* v_cv  + ... 
  -2.86820202211694E+01 .*       u_cv .* RUD_cv  + ... 
  -2.39686331072052E+01 .*         v_cv .* w_cv  + ... 
  -2.31832314141299E+01 .*        w_cv .* N5_cv  + ... 
  +2.35506573968857E+01 .*        w_cv .* N3_cv  + ... 
  +1.88771114397595E+01 .*        u_cv .* N6_cv  + ... 
  -6.47663900342239E+01 .*             N3_cv.^2  + ... 
  +6.63752527620345E+01 .*             N5_cv.^2  + ... 
  -1.57471080977158E+01 .*        w_cv .* N4_cv  + ... 
  -1.42693433960184E+01 .*        u_cv .* N4_cv  + ... 
  +1.51770140649063E+01 .*        w_cv .* N6_cv  + ... 
  -1.22920245580326E+01 .*        v_cv .* N8_cv  + ... 
  +1.33216354100371E+01 .*       N3_cv .* N4_cv  + ... 
  -1.43776663516998E+01 .*        v_cv .* N2_cv  + ... 
  +1.25157797727103E+01 .*       w_cv .* RUD_cv  + ... 
  +1.15917343266445E+01 .*       LA_cv .* N7_cv  + ... 
  -2.71176390251187E+01 .*             N4_cv.^2  + ... 
  -1.06241836003055E+01 .*       N3_cv .* N7_cv  + ... 
  -1.03957214039422E+01 .*        v_cv .* N7_cv  + ... 
  +3.50642753396202E+01 .*             N3_cv.^3  + ... 
  +1.76855122680831E+01 .*             N6_cv.^2  + ... 
  -9.81445263245855E+00 .*       N6_cv .* N7_cv  + ... 
  -9.65941072261516E+00 .*        v_cv .* N1_cv  + ... 
  -1.00887073930092E+01 .*       N5_cv .* N6_cv  + ... 
  +1.04243812092793E+01 .*        w_cv .* N7_cv  + ... 
  -9.95506781567286E+00 .*             N2_cv.^3  + ... 
  -9.74117251108992E+00 .*        v_cv .* N4_cv  + ... 
  -8.59119399621313E+00 .*                N1_cv  + ... 
  +9.65216358795503E+00 .*      RUD_cv .* N1_cv  + ... 
  -1.01312347903212E+01 .*        w_cv .* N1_cv  + ... 
  -9.35645048969002E+00 .*      LA_cv .* RUD_cv  + ... 
  -9.15187091828390E+00 .*       RA_cv .* N5_cv  + ... 
  +9.28965527837581E+00 .*       N7_cv .* N8_cv  + ... 
  +9.17788778974122E+00 .*       RE_cv .* N5_cv  + ... 
  -9.05337873458066E+00 .*       N2_cv .* N4_cv  + ... 
  -8.64437236905079E+00 .*        w_cv .* N2_cv ;      

% Fnormal Model

Fnormal = ...
  +8.87282693760624E+03 .*                    1  + ... 
  +1.20135740290698E+03 .*                 w_cv  + ... 
  +1.16896293316574E+03 .*                N4_cv  + ... 
  +1.18708606355321E+03 .*                N6_cv  + ... 
  +1.11247435932502E+03 .*                N2_cv  + ... 
  +1.10659288539594E+03 .*                N8_cv  + ... 
  +7.62254065533589E+02 .*                N7_cv  + ... 
  +7.53648908830186E+02 .*                N1_cv  + ... 
  +6.68845079411481E+02 .*                N3_cv  + ... 
  +6.72063052336578E+02 .*                N5_cv  + ... 
  +4.48805778097462E+02 .*                 u_cv  + ... 
  +1.95403926659180E+02 .*              w_cv.^2  + ... 
  +3.08955547705547E+02 .*         u_cv .* w_cv  + ... 
  +1.69301095225899E+02 .*             N6_cv.^2  + ... 
  +1.74149998049239E+02 .*             N8_cv.^2  + ... 
  +1.26957381814228E+02 .*                LA_cv  + ... 
  +1.21174980996445E+02 .*                RA_cv  + ... 
  +1.64011232937325E+02 .*             N7_cv.^2  + ... 
  +1.21588457591342E+02 .*        w_cv .* N6_cv  + ... 
  +1.09416761199758E+02 .*        w_cv .* N4_cv  + ... 
  +1.03362934942525E+02 .*        u_cv .* N6_cv  + ... 
  +1.82776821391500E+02 .*             N3_cv.^2  + ... 
  +1.02634053511110E+02 .*        u_cv .* N4_cv  + ... 
  +8.97009057131097E+01 .*        w_cv .* N2_cv  + ... 
  +1.63155383357721E+02 .*             N2_cv.^2  + ... 
  +8.16436469158500E+01 .*        w_cv .* N8_cv  + ... 
  +1.34783257417982E+02 .*             N4_cv.^2  + ... 
  -1.43355327467195E+02 .*              v_cv.^2  + ... 
  +1.42837315870936E+02 .*             N1_cv.^2 ;      

% Mroll Model

Mroll = ...
  -1.98643036611109E+04 .*                N8_cv  + ... 
  +1.96163531306679E+04 .*                N2_cv  + ... 
  +1.37623009913244E+04 .*                N1_cv  + ... 
  -1.37343187734553E+04 .*                N7_cv  + ... 
  -7.71145055842772E+03 .*                N6_cv  + ... 
  +7.88602408783047E+03 .*                N4_cv  + ... 
  +4.66241862363265E+03 .*                N3_cv  + ... 
  -4.71171463206559E+03 .*                N5_cv  + ... 
  +1.73632516970305E+03 .*                LA_cv  + ... 
  -1.71348299516345E+03 .*                RA_cv  + ... 
  -1.71399484896984E+03 .*         v_cv .* w_cv  + ... 
  -1.18529227636964E+03 .*                 v_cv  + ... 
  -1.03930763503581E+03 .*        w_cv .* N1_cv  + ... 
  +9.93536227491809E+02 .*        w_cv .* N7_cv  + ... 
  +9.37435233761027E+02 .*        u_cv .* LA_cv  + ... 
  -9.37632800744642E+02 .*        w_cv .* N8_cv  + ... 
  -8.32548403739681E+02 .*        u_cv .* RA_cv  + ... 
  +6.85471441461560E+02 .*        w_cv .* N2_cv  + ... 
  +3.52852549718517E+03 .*             N2_cv.^2  + ... 
  -3.42500957190401E+03 .*             N8_cv.^2  + ... 
  -2.96420172129674E+03 .*             N7_cv.^2  + ... 
  +2.89583980057180E+03 .*             N1_cv.^2 ;      

% Mpitch Model

Mpitch = ...
  +3.68386886679220E+03 .*                N3_cv  + ... 
  +3.68727436000804E+03 .*                N5_cv  + ... 
  +3.39497623262367E+03 .*                N7_cv  + ... 
  +3.38521729256396E+03 .*                N1_cv  + ... 
  -3.40470982416229E+03 .*                N2_cv  + ... 
  -3.37783527528972E+03 .*                N8_cv  + ... 
  +2.88225851003144E+03 .*                    1  + ... 
  -2.91755256344785E+03 .*                N6_cv  + ... 
  -2.88842859286586E+03 .*                N4_cv  + ... 
  -7.91123300976865E+02 .*             N2_cv.^2  + ... 
  -3.22153415303863E+02 .*                RE_cv  + ... 
  +3.42361431800527E+02 .*              u_cv.^3  + ... 
  +3.38487621821177E+02 .*        w_cv .* N3_cv  + ... 
  +3.53976944430379E+02 .*        w_cv .* N5_cv  + ... 
  -2.69833866318930E+02 .*                LE_cv  + ... 
  -2.79177103186949E+02 .*        v_cv .* N5_cv  + ... 
  +3.07163264246614E+02 .*        v_cv .* N3_cv  + ... 
  -2.48407992851214E+02 .*        v_cv .* N4_cv  + ... 
  +5.67576144296002E+02 .*             N5_cv.^2  + ... 
  -8.09069345162211E+02 .*             N8_cv.^2  + ... 
  +5.77660951037590E+02 .*             N1_cv.^2  + ... 
  -2.60829015107695E+02 .*         u_cv .* w_cv  + ... 
  -6.18106266161914E+02 .*             N4_cv.^2  + ... 
  +5.13931233318378E+02 .*             N7_cv.^2  + ... 
  -2.19921662964601E+02 .*        u_cv .* LE_cv  + ... 
  -2.11585394940918E+02 .*        u_cv .* RE_cv  + ... 
  +1.64122010307063E+02 .*                 w_cv  + ... 
  +1.96485832816910E+02 .*        v_cv .* N6_cv  + ... 
  +4.65896302743280E+02 .*              w_cv.^2  + ... 
  -4.55117130910195E+02 .*             N6_cv.^2 ;      

% Myaw Model

Myaw = ...
  +1.95592094975335E+03 .*                N4_cv  + ... 
  +1.88735141319313E+03 .*                N5_cv  + ... 
  -1.94771565856483E+03 .*                N3_cv  + ... 
  -1.90921103450052E+03 .*                N6_cv  + ... 
  +1.51367690203445E+03 .*                N8_cv  + ... 
  -1.47831847868129E+03 .*                N2_cv  + ... 
  +7.37842363093256E+02 .*               RUD_cv  + ... 
  +5.17066072655507E+02 .*       u_cv .* RUD_cv  + ... 
  +3.57373825310185E+02 .*        u_cv .* N4_cv  + ... 
  -3.46783129074005E+02 .*        v_cv .* N5_cv  + ... 
  -3.08518727863116E+02 .*        v_cv .* N3_cv  + ... 
  -2.71843929584420E+02 .*        u_cv .* N1_cv  + ... 
  -2.79672247960316E+02 .*        u_cv .* N6_cv  + ... 
  +2.76269914291945E+02 .*        u_cv .* N7_cv  + ... 
  +2.74693070287690E+02 .*        w_cv .* N4_cv  + ... 
  -2.14297188629469E+02 .*        w_cv .* N6_cv  + ... 
  +2.34069704353212E+02 .*        v_cv .* N4_cv  + ... 
  -2.05404927322277E+02 .*       w_cv .* RUD_cv  + ... 
  +2.19086528675574E+02 .*        v_cv .* N6_cv  + ... 
  +2.00342635541489E+02 .*         v_cv .* w_cv  + ... 
  -1.64596574001635E+02 .*                LA_cv  + ... 
  +1.89778303529204E+02 .*       N5_cv .* N6_cv  + ... 
  +6.17715872861260E+02 .*             N8_cv.^2  + ... 
  -5.90349547095267E+02 .*             N3_cv.^2  + ... 
  -5.72229896921965E+02 .*             N2_cv.^2  + ... 
  +4.77675140094754E+02 .*             N5_cv.^2  + ... 
  +1.47067943924790E+02 .*        u_cv .* N8_cv  + ... 
  +3.88731936139572E+02 .*             RA_cv.^2  + ... 
  -4.00275097643810E+02 .*             N6_cv.^2  + ... 
  +1.25980675363044E+02 .*             RA_cv.^3  + ... 
  -1.35903165394832E+02 .*       N2_cv .* N3_cv  + ... 
  -1.24967080253126E+02 .*        w_cv .* N1_cv  + ... 
  -1.35579320639314E+02 .*       N3_cv .* N4_cv  + ... 
  -1.21928515923829E+02 .*        u_cv .* RE_cv  + ... 
  -1.26077535656917E+02 .*       RA_cv .* N2_cv  + ... 
  +1.04629643044518E+02 .*                LE_cv  + ... 
  -1.07568412584019E+02 .*       N4_cv .* N7_cv  + ... 
  +1.31933477308622E+02 .*                    1  + ... 
  +1.03832765244587E+02 .*       N4_cv .* N8_cv  + ... 
  -9.96606588282909E+01 .*       RE_cv .* N5_cv  + ... 
  -9.42880248816848E+01 .*       N2_cv .* N6_cv  + ... 
  +9.31464079537483E+01 .*        v_cv .* N2_cv ;      

% Flift Model

Flift = ...
  +8.90812225876903E+03 .*                    1  + ... 
  +1.12453454071561E+03 .*                N4_cv  + ... 
  +1.14390830014245E+03 .*                N6_cv  + ... 
  +1.06623146830463E+03 .*                N2_cv  + ... 
  +1.04961665980513E+03 .*                N8_cv  + ... 
  +1.03072030906540E+03 .*                 w_cv  + ... 
  +7.24534402667982E+02 .*                N7_cv  + ... 
  +7.24576330660154E+02 .*                 u_cv  + ... 
  +7.19481688624005E+02 .*                N1_cv  + ... 
  +6.38129521303673E+02 .*                N5_cv  + ... 
  +6.33457273529289E+02 .*                N3_cv  + ... 
  +3.20753529772084E+02 .*         u_cv .* w_cv  + ... 
  +1.85900582217132E+02 .*             N7_cv.^2  + ... 
  +1.28270071851636E+02 .*                LA_cv  + ... 
  +1.30014290011250E+02 .*        u_cv .* N4_cv  + ... 
  +1.81612145037772E+02 .*             N3_cv.^2  + ... 
  +1.36659753584708E+02 .*        u_cv .* N6_cv  + ... 
  +1.16689782223692E+02 .*                RA_cv  + ... 
  +1.09904301372254E+02 .*        w_cv .* N6_cv  + ... 
  +9.90311039275785E+01 .*        w_cv .* N4_cv  + ... 
  -3.36872973647805E+02 .*              w_cv.^2  + ... 
  +1.61056748315633E+02 .*             N2_cv.^2  + ... 
  +1.37426509478769E+02 .*             N8_cv.^2  + ... 
  +8.53396868123899E+01 .*        u_cv .* N2_cv  + ... 
  +7.97190088384760E+01 .*        u_cv .* N8_cv  + ... 
  +7.42762048044546E+01 .*        v_cv .* N3_cv  + ... 
  +7.41925623172130E+01 .*        u_cv .* RA_cv  + ... 
  +7.08401196924528E+01 .*        u_cv .* LA_cv  + ... 
  +1.28844962611167E+02 .*             N4_cv.^2  + ... 
  +1.27428538849978E+02 .*             N6_cv.^2 ;      

% Fdrag Model

Fdrag = ...
  +2.90344542425375E+03 .*                 w_cv  + ... 
  +3.95717656261927E+02 .*                    1  + ... 
  -9.08385137944767E+02 .*         u_cv .* w_cv  + ... 
  +3.23638285024368E+02 .*        w_cv .* N2_cv  + ... 
  +3.38220493802001E+02 .*        w_cv .* N6_cv  + ... 
  +3.32653873337336E+02 .*        w_cv .* N8_cv  + ... 
  +3.41275199876057E+02 .*        w_cv .* N4_cv  + ... 
  +2.45537653150739E+02 .*        w_cv .* N1_cv  + ... 
  +2.32097929415364E+02 .*        w_cv .* N7_cv  + ... 
  +3.57669883990841E+02 .*              w_cv.^2  + ... 
  +2.17578210913018E+02 .*        w_cv .* N5_cv  + ... 
  +2.15788163203217E+02 .*        w_cv .* N3_cv  + ... 
  +1.38986796659425E+02 .*                 u_cv  + ... 
  +9.48813058323239E+01 .*        v_cv .* N3_cv  + ... 
  +6.94490899746495E+01 .*                N2_cv  + ... 
  +7.26901938577641E+01 .*             N8_cv.^3  + ... 
  +5.91813856250063E+01 .*             N4_cv.^3 ;      






% Output response variable predictions
C = [Faxial,Fside,Fnormal,Mroll,Mpitch,Myaw,Flift,Fdrag];

return