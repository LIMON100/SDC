%Operating frequency (Hz)
fc = 77.0e9;

%Transmitted power (W)
Pt = 3e-3;

%Antenna Gain (linear)
G =  10000;

%Minimum Detectable Power
Ps = 1e-10;

%RCS of a car
RCS = 100;

%Speed of light
c = 3*10^8;

%Minimum Detectable Power
Pe = 1e-10;

%TODO: Calculate the wavelength
lamda = c/fc;
fprintf('wavelength = %.3f mm\n', lamda * 1000);

%TODO : Measure the Maximum Range a Radar can see.
R = nthroot((Ps * G^2 * lamda^2 * RCS / (Pe * (4 * pi)^3)), 4);
fprintf('Max Detection range = %.3f m\n', R);