longstr1 = '1 25544U 98067A   13192.54519036  .00002514  00000-0  52449-4 0  1643';
longstr2 = '2 25544  51.6493 347.6029 0005392 172.1766 232.4290 15.49762614838488';
SGP4_Setup(longstr1, longstr2)

JD = juliandate(clock + [0 0 0 4 0 0]);
tsince = JD2Tsince(JD);
[r, v] = sgp4(tsince);
r = r*1000;
v = v*1000;
lla = ecef2lla(r');
lla(1:2)