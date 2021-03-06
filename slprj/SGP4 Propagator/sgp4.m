% -----------------------------------------------------------------------------
%
%                              procedure sgp4
%
%  this procedure is the sgp4 prediction model from space command. this is an
%    updated and combined version of sgp4 and sdp4, which were originally
%    published separately in spacetrack report #3. this version follows the
%    methodology from the aiaa paper (2006) describing the history and
%    development of the code.
%
% Author:
%   Jeff Beck
%   beckja@alumni.lehigh.edu
%    current :
%               7 may 08  david vallado
%                           update small eccentricity check
%    changes :
%              16 nov 07  david vallado
%                           misc fixes for better compliance
%   1.0 (aug 7, 2006) - update for paper dav
% original comments from Vallado C++ version:
%   author        : david vallado                  719-573-2600   28 jun 2005
%
%   inputs        :
%     satrec    - initialised structure from sgp4init() call.
%     tsince    - time eince epoch (minutes)
%
%   outputs       :
%     r           - position vector                     km
%     v           - velocity                            km/sec
%     return code - non-zero on error.
%                    1 - mean elements, ecc >= 1.0 or ecc < -0.001 or a < 0.95 er
%                    2 - mean motion less than 0.0
%                    3 - pert elements, ecc < 0.0  or  ecc > 1.0
%                    4 - semi-latus rectum < 0.0
%                    5 - epoch elements are sub-orbital
%                    6 - satellite has decayed
%
%   locals        :
%     am          -
%     axnl, aynl        -
%     betal       -
%     COSIM   , SINIM   , COSOMM  , SINOMM  , Cnod    , Snod    , Cos2u   ,
%     Sin2u   , Coseo1  , Sineo1  , Cosi    , Sini    , Cosip   , Sinip   ,
%     Cosisq  , Cossu   , Sinsu   , Cosu    , Sinu
%     Delm        -
%     Delomg      -
%     Dndt        -
%     Eccm        -
%     EMSQ        -
%     Ecose       -
%     El2         -
%     Eo1         -
%     Eccp        -
%     Esine       -
%     Argpm       -
%     Argpp       -
%     Omgadf      -
%     Pl          -
%     R           -
%     RTEMSQ      -
%     Rdotl       -
%     Rl          -
%     Rvdot       -
%     Rvdotl      -
%     Su          -
%     T2  , T3   , T4    , Tc
%     Tem5, Temp , Temp1 , Temp2  , Tempa  , Tempe  , Templ
%     U   , Ux   , Uy    , Uz     , Vx     , Vy     , Vz
%     inclm       - inclination
%     mm          - mean anomaly
%     nm          - mean motion
%     nodem      - longi of ascending node
%     xinc        -
%     xincp       -
%     xl          -
%     xlm         -
%     mp          -
%     xmdf        -
%     xmx         -
%     xmy         -
%     nodedf     -
%     xnode       -
%     nodep      -
%     np          -
%
%   coupling      :
%     getgravconst
%     dpper
%     dspace
%
%   references    :
%     hoots, roehrich, norad spacetrack report #3 1980
%     hoots, norad spacetrack report #6 1986
%     hoots, schumacher and glover 2004
%     vallado, crawford, hujsak, kelso  2006
%  ----------------------------------------------------------------------------*/

function r = sgp4(tsince)
r = [0 0 0];

%% Define Constants
twopi = 2.0 * pi;
x2o3  = 2.0 / 3.0;
% sgp4fix divisor for divide by zero check on inclination
% the old check used 1.0 + cos(pi-1.0e-9), but then compared it to
% 1.5 e-12, so the threshold was changed to 1.5e-12 for consistancy
temp4    =   1.5e-12;

%  sgp4fix identify constants and allow alternate values
global satrec gravc
vkmpersec     = gravc.radiusearthkm * gravc.xke/60.0;

% Clear sgp4 error flag
satrec.t     = tsince;
satrec.error = 0;
mrt = 0.0;

% Update for secular gravity and atmospheric drag
xmdf    = satrec.mo + satrec.mdot * satrec.t;
argpdf  = satrec.argpo + satrec.argpdot * satrec.t;
nodedf  = satrec.nodeo + satrec.nodedot * satrec.t;
argpm   = argpdf;
mm      = xmdf;
t2      = satrec.t * satrec.t;
nodem   = nodedf + satrec.nodecf * t2;
tempa   = 1.0 - satrec.cc1 * satrec.t;
tempe   = satrec.bstar * satrec.cc4 * satrec.t;
templ   = satrec.t2cof * t2;

if (satrec.isimp ~= 1) % Not deep space model
    delomg = satrec.omgcof * satrec.t;
    delm   = satrec.xmcof *...
        ((1.0 + satrec.eta * cos(xmdf))^3 -...
        satrec.delmo);
    temp   = delomg + delm;
    mm     = xmdf + temp;
    argpm  = argpdf - temp;
    t3     = t2 * satrec.t;
    t4     = t3 * satrec.t;
    tempa  = tempa - satrec.d2 * t2 - satrec.d3 * t3 -...
        satrec.d4 * t4;
    tempe  = tempe + satrec.bstar * satrec.cc5 * (sin(mm) -...
        satrec.sinmao);
    templ  = templ + satrec.t3cof * t3 + t4 * (satrec.t4cof +...
        satrec.t * satrec.t5cof);
end

nm    = satrec.no;
em    = satrec.ecco;
inclm = satrec.inclo;

if (nm <= 0.0)
    satrec.error = 2;
end

am = (gravc.xke / nm)^x2o3 * tempa * tempa;
nm = gravc.xke / am^1.5;
em = em - tempe;

% fix tolerance for error recognition
if ((em >= 1.0) || (em < -0.001) || (am < 0.95))
    satrec.error = 1;
end
%   sgp4fix change test condition for eccentricity
if (em < 1.0e-6)
    em  = 1.0e-6;
end
mm     = mm + satrec.no * templ;
xlm    = mm + argpm + nodem;
emsq   = em * em;
temp   = 1.0 - emsq;
nodem  = rem(nodem, twopi);
argpm  = rem(argpm, twopi);
xlm    = rem(xlm, twopi);
mm     = rem(xlm - argpm - nodem, twopi);

% compute extra mean quantities
sinim = sin(inclm);
cosim = cos(inclm);

% add lunar-solar periodics
ep     = em;
xincp  = inclm;
argpp  = argpm;
nodep  = nodem;
mp     = mm;
sinip  = sinim;
cosip  = cosim;

% long period periodics
axnl = ep * cos(argpp);
temp = 1.0 / (am * (1.0 - ep * ep));
aynl = ep* sin(argpp) + temp * satrec.aycof;
xl   = mp + argpp + nodep + temp * satrec.xlcof * axnl;

% solve kepler's equation
u    = rem(xl - nodep, twopi);
eo1  = u;
tem5 = 9999.9;
ktr = 1;

% sgp4fix for kepler iteration
% the following iteration needs better limits on corrections

% To please the compiler, the following variable are defined here so that
% they are not undefined for some execution paths.
sineo1 = 0.0;
coseo1 = 0.0;
while (( abs(tem5) >= 1.0e-12) && (ktr <= 10) )
    sineo1 = sin(eo1);
    coseo1 = cos(eo1);
    tem5   = 1.0 - coseo1 * axnl - sineo1 * aynl;
    tem5   = (u - aynl * coseo1 + axnl * sineo1 - eo1) / tem5;
    if(abs(tem5) >= 0.95)
        if tem5 > 0.0
            tem5 = 0.95;
        else
            tem5 = -0.95;
        end
    end
    eo1    = eo1 + tem5;
    ktr = ktr + 1;
end

% Short period preliminary quantities
ecose = axnl*coseo1 + aynl*sineo1;
esine = axnl*sineo1 - aynl*coseo1;
el2   = axnl*axnl + aynl*aynl;
pl    = am*(1.0-el2);
if (pl < 0.0)
    satrec.error = 4;
    r = [0; 0; 0];
else
    rl     = am * (1.0 - ecose);
    rdotl  = sqrt(am) * esine/rl;
    rvdotl = sqrt(pl) / rl;
    betal  = sqrt(1.0 - el2);
    temp   = esine / (1.0 + betal);
    sinu   = am / rl * (sineo1 - aynl - axnl * temp);
    cosu   = am / rl * (coseo1 - axnl + aynl * temp);
    su     = atan2(sinu, cosu);
    sin2u  = (cosu + cosu) * sinu;
    cos2u  = 1.0 - 2.0 * sinu * sinu;
    temp   = 1.0 / pl;
    temp1  = 0.5 * gravc.j2 * temp;
    temp2  = temp1 * temp;
    
    % update for short period periodics
    mrt   = rl * (1.0 - 1.5 * temp2 * betal * satrec.con41) +...
        0.5 * temp1 * satrec.x1mth2 * cos2u;
    su    = su - 0.25 * temp2 * satrec.x7thm1 * sin2u;
    xnode = nodep + 1.5 * temp2 * cosip * sin2u;
    xinc  = xincp + 1.5 * temp2 * cosip * sinip * cos2u;
    mvt   = rdotl - nm * temp1 * satrec.x1mth2 * sin2u / gravc.xke;
    rvdot = rvdotl + nm * temp1 * (satrec.x1mth2 * cos2u +...
        1.5 * satrec.con41) / gravc.xke;
    
    % orientation vectors
    sinsu =  sin(su);
    cossu =  cos(su);
    snod  =  sin(xnode);
    cnod  =  cos(xnode);
    sini  =  sin(xinc);
    cosi  =  cos(xinc);
    xmx   = -snod * cosi;
    xmy   =  cnod * cosi;
    ux    =  xmx * sinsu + cnod * cossu;
    uy    =  xmy * sinsu + snod * cossu;
    uz    =  sini * sinsu;
    vx    =  xmx * cossu - cnod * sinsu;
    vy    =  xmy * cossu - snod * sinsu;
    vz    =  sini * cossu;
      
    % position and velocity (in km and km/sec)
    r = zeros(3,1);
    v = zeros(3,1);
    r(1,1) = (mrt * ux)* gravc.radiusearthkm;
    r(2,1) = (mrt * uy)* gravc.radiusearthkm;
    r(3,1) = (mrt * uz)* gravc.radiusearthkm;
%     v(1,1) = (mvt * ux + rvdot * vx) * vkmpersec;
%     v(2,1) = (mvt * uy + rvdot * vy) * vkmpersec;
%     v(3,1) = (mvt * uz + rvdot * vz) * vkmpersec;
    
    % Calculate the GMST1982 angle
    TUT1 = (satrec.jdsatepoch + tsince/1440 - 2451545.0)/36525; % 1440 minutes in a day
    TGMST_1982 = 67310.54841 + (876600*3600 + 8640184.812866)*TUT1 + 0.093104*TUT1^2 - 6.2E-6*TUT1^3;
    TGMST_1982 = (mod(TGMST_1982,86400)/240)*pi/180; % 1 sec = 1/240 degrees
    ROT3 = [cos(TGMST_1982) -sin(TGMST_1982) 0;...
        sin(TGMST_1982) cos(TGMST_1982) 0;...
        0 0 1];
    
    r = ROT3'*r;
%     LLA = convert_ecef2lla(r'*1000);
%     w_e = [0; 0; 0.00007292115855300]; % rad/solar second
%     v = ROT3'*v + cross(w_e,r);
    
end % // if pl > 0

% // sgp4fix for decaying satellites
if (mrt < 1.0)
    satrec.error = 6;
end

% global idebug dbgfile
% if idebug
%     debug7;
% end

return;

end

% function LLA = convert_ecef2lla(r)
% Re = 6371*1000; % m
% R = sqrt(sum(r.^2));
% xy = sqrt(r(1)^2 + r(2)^2);
% lon = -acosd(r(1)/xy);
% lat = acosd(xy/R);
% alt = R - Re;
% LLA = [lat lon alt];
% end
