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

/// @brief Specific gas constant for dry air [J/kg K]
constexpr double R = 287.05;
/// @brief Aproximation for the gravity by the ISA [m/s²]
constexpr double g0 = 9.807;
/// @brief  Aproximation for the Mean Sea Level Temp by the ISA [K]
constexpr double T0 = 288.15;
/// @brief  Aproximation for the Mean Sea Level Pressure by the ISA [hPA]
constexpr double P0 = 1013.25; 
/// @brief  Aproximation for the Lapse Rate by the ISA [K/m]
constexpr double L = 0.0065;

/// @brief Calculates the standard atmosphere total pressure
/// @param[in] altitudeMSL Geodetic height above MSL (mean sea level) [m]
/// @return The total pressure in [hPa] = [mbar]
/// @note See \cite RTKLIB RTKLIB ch. E.5, eq. E.5.1, p. 149
[[nodiscard]] double calcTotalPressureStAtm(double altitudeMSL);


/// @brief Calculates the standard atmosphere Height above MSL (mean sea level) [m]
/// @param[in] pressure total pressure in [hPa] = [mbar]
/// @return Geopotential Height above MSL (mean sea level) [m]
[[nodiscard]] double calcHeightStAtm(double pressure);


/// @brief Calculates the standard atmosphere Height above MSL (mean sea level) [m] (Calibrated for Temp and Presure Principle Error)
/// @param[in] pressure total pressure in [hPa] = [mbar]
/// @param[in] temp_start at the start of the Measurment in [K]
/// @param[in] pres_start  pressure at the start of the Measurment in [hPa] = [mbar]
/// @param[in] H_start Geopotential Height from the Start point  above MSL (mean sea level) [m]
/// @return Geopotential Height above MSL (mean sea level) [m]
[[nodiscard]]  double calcCalibrateHeightStAtm(double pressure, double temp_start, double pres_start, double H_start);

/// @brief Calculates the standard atmosphere Mean Sea Level pressure
/// @param[in] altitudeMSL Geodetic height above MSL (mean sea level) [m]
/// @param[in] pressure_meas total pressure in [hPa] = [mbar]
/// @param[in] T_0 sea level temp [K]
/// @return The Mean Sea Level pressure in [hPa] = [mbar]
[[nodiscard]] double calcMeanSeaLevelPressure(double altitudeMSL, double pressure_meas, double T_0);


} // namespace NAV
