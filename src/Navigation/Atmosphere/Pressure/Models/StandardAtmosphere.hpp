// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file StandardAtmosphere.hpp
/// @brief Standard Atmosphere pressure model
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2023-01-31

#pragma once

#include <gcem.hpp>

namespace NAV
{

/// @brief Calculates the standard atmosphere total pressure
/// @param[in] altitudeMSL Geodetic height above MSL (mean sea level) [m]
/// @return The total pressure in [hPa] = [mbar] ???
/// @note See \cite RTKLIB RTKLIB ch. E.5, eq. E.5.1, p. 149
[[nodiscard]] constexpr double calcTotalPressureStAtm(double altitudeMSL)
{
    return 1013.25 * gcem::pow(1 - 2.2557e-5 * altitudeMSL, 5.2568);
}

/// @brief Calculates the standard atmosphere Height above MSL (mean sea level) [m]
/// @param[in] pressure total pressure in [hPa] = [mbar]
/// @return Geopotential Height above MSL (mean sea level) [m]
[[nodiscard]] constexpr double calcHeightStAtm(double pressure)
{
    return 44.300 * (1 - gcem::pow(pressure / 1013.25, 0.19)) * 1000;
}

/// @brief Calculates the standard atmosphere Height above MSL (mean sea level) [m] (Calibrated for Temp and Presure Principle Error)
/// @param[in] pressure total pressure in [hPa] = [mbar]
/// @param[in] temp_start at the start of the Measurment in [K]
/// @param[in] pressure_start  pressure at the start of the Measurment in [hPa] = [mbar]
/// @param[in] H_start Geopotential Height from the Start point  above MSL (mean sea level) [m]
/// @return Geopotential Height above MSL (mean sea level) [m]
[[nodiscard]] constexpr double calcCalibrateHeightStAtm(double pressure, double temp_start, double pres_start, double H_start)
{
    return (-temp_start / 0.0065 * (1 - gcem::pow(pressure / pres_start, 0.19))) + H_start;
}

/// @brief Calculates the standard atmosphere Mean Sea Level pressure
/// @param[in] altitudeMSL Geodetic height above MSL (mean sea level) [m]
/// @param[in] pressure_meas total pressure in [hPa] = [mbar]
/// @param[in] T_0 sea level temp [K]
/// @return The Mean Sea Level pressure in [hPa] = [mbar]
[[nodiscard]] constexpr double calcMeanSeaLevelPressure(double altitudeMSL, double pressure_meas, double T_0)
{
    return pressure_meas / (gcem::pow(1 - (0.0065 / T_0) * altitudeMSL, (9.81 * 0.0289) / (8.2144 * 0.0065)));
}

} // namespace NAV
