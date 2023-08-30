#include <iostream>
#include <cmath>

int main() {
    // Operating frequency (Hz)
    double fc = 77.0e9;

    // Transmitted power (W)
    double Pt = 3e-3;

    // Antenna Gain (linear)
    double G = 10000;

    // Minimum Detectable Power
    double Ps = 1e-10;

    // RCS of a car
    double RCS = 100;

    // Speed of light
    double c = 3e8;

    // Minimum Detectable Power
    double Pe = 1e-10;

    // Calculate the wavelength
    double lamda = c / fc;
    std::cout << "wavelength = " << lamda * 1000 << " mm" << std::endl;

    // Calculate the Maximum Range a Radar can see
    double R = std::pow((Ps * std::pow(G, 2) * std::pow(lamda, 2) * RCS) / (Pe * std::pow(4 * M_PI, 3)), 0.25);
    std::cout << "Max Detection range = " << R << " m" << std::endl;

    return 0;
}
