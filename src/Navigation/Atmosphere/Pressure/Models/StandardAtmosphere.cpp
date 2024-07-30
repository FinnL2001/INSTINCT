#include <gcem.hpp>
#include "Navigation/Atmosphere/Pressure/Models/StandardAtmosphere.hpp"

namespace NAV
{
[[nodiscard]] double calcTotalPressureStAtm(double altitudeMSL)
{
    return P0 * gcem::pow(1 - (L/T0) * altitudeMSL, (g0 / (L * R)));
}


[[nodiscard]] double calcHeightStAtm(double pressure)
{
    return T0 / L * (1 - gcem::pow(pressure / P0, (L * R / g0)));
}

[[nodiscard]] double calcCalibrateHeightStAtm(double pressure, double temp_start, double pres_start, double H_start)
{
    return (temp_start / L * (1 - gcem::pow(pressure / pres_start, (L * R / g0)))) + H_start;
}

[[nodiscard]] double calcMeanSeaLevelPressure(double altitudeMSL, double pressure_meas, double T_0)
{
    return pressure_meas / (gcem::pow(1 - (L / T_0) * altitudeMSL, (g0 ) / (R * L)));
}


} // namespace NAV