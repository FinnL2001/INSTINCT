/// @file InsMechanization.hpp
/// @brief Inertial Navigation Mechanization Functions
/// @author T. Topp (thomas.topp@nav.uni-stuttgart.de)
/// @date 2020-09-02

#pragma once

#include "util/LinearAlgebra.hpp"

namespace NAV
{
/// @brief Integrates the angular velocities and calculates the new Quaternion q_ep with Runge Kutta of 3rd Order
/// @param[in] timeDifferenceSec__t0 Δtₖ Time difference in [seconds]. This epoch to previous epoch
/// @param[in] timeDifferenceSec__t1 Δtₖ₋₁ Time difference in [seconds]. Previous epoch to twice previous epoch
/// @param[in] angularVelocity_ip_p__t0 ω_ip_p (tₖ) Angluar velocity in [rad/s],
///                                     of the inertial to platform system, in platform coordinates, at the time tₖ
/// @param[in] angularVelocity_ip_p__t1 ω_ip_p (tₖ₋₁) Angluar velocity in [rad/s],
///                                     of the inertial to platform system, in platform coordinates, at the time tₖ₋₁
/// @param[in] angularVelocity_ie_e__t0 ω_ie_e (tₖ) Angluar velocity in [rad/s],
///                                     of the inertial to earth system, in earth coordinates, at the time tₖ
/// @param[in] quaternion_ep__t1 q (tₖ₋₁) Quaternion, from platform to earth coordinates, at the time tₖ₋₁
/// @param[in] quaternion_ep__t2 q (tₖ₋₂) Quaternion, from platform to earth coordinates, at the time tₖ₋₂
/// @return The updated Quaternion q_ep
///
/// @note See C. Jekeli (2001) - Inertial Navigation Systems with Geodetic Applications (Chapter 4.2.4.1.2)
///       See T. Hobiger - Inertialnavigation (Lecture Slides Chapter 8)
[[nodiscard]] Quaterniond<Earth, Platform> updateQuaternion_ep_RungeKutta3(const long double& timeDifferenceSec__t0,
                                                                           const long double& timeDifferenceSec__t1,
                                                                           const Vector3d<Platform>& angularVelocity_ip_p__t0,
                                                                           const Vector3d<Platform>& angularVelocity_ip_p__t1,
                                                                           const Vector3d<Earth>& angularVelocity_ie_e__t0,
                                                                           const Quaterniond<Earth, Platform>& quaternion_ep__t1,
                                                                           const Quaterniond<Earth, Platform>& quaternion_ep__t2);

/// @brief Integrates the angular velocities and calculates the new Quaternion q_nb with Runge Kutta of 3rd Order
/// @param[in] timeDifferenceSec__t0 Δtₖ Time difference in [seconds]. This epoch to previous epoch
/// @param[in] timeDifferenceSec__t1 Δtₖ₋₁ Time difference in [seconds]. Previous epoch to twice previous epoch
/// @param[in] angularVelocity_ip_b__t0 ω_ip_b (tₖ) Angluar velocity in [rad/s],
///                                     of the inertial to platform system, in body coordinates, at the time tₖ
/// @param[in] angularVelocity_ip_b__t1 ω_ip_b (tₖ₋₁) Angluar velocity in [rad/s],
///                                     of the inertial to platform system, in body coordinates, at the time tₖ₋₁
/// @param[in] angularVelocity_ie_n__t1 ω_ie_n (tₖ₋₁) Angluar velocity in [rad/s]
///                                     of the inertial to earth system, in navigation coordinates, at the time tₖ₋₁
/// @param[im] angularVelocity_en_n__t1 ω_en_n (tₖ₋₁) Transport Rate, rotation rate of the Earth frame relative to the navigation frame
///                                     in navigation coordinates
/// @param[in] quaternion_nb__t1 q (tₖ₋₁) Quaternion, from body to navigation coordinates, at the time tₖ₋₁
/// @param[in] quaternion_nb__t2 q (tₖ₋₂) Quaternion, from body to navigation coordinates, at the time tₖ₋₂
/// @return The updated Quaternion q_ep
///
/// @note See C. Jekeli (2001) - Inertial Navigation Systems with Geodetic Applications (Chapter 4.2.4.1.2)
///       See T. Hobiger - Inertialnavigation (Lecture Slides Chapter 8)
[[nodiscard]] Quaterniond<Navigation, Body> updateQuaternion_nb_RungeKutta3(const long double& timeDifferenceSec__t0,
                                                                            const long double& timeDifferenceSec__t1,
                                                                            const Vector3d<Body>& angularVelocity_ip_b__t0,
                                                                            const Vector3d<Body>& angularVelocity_ip_b__t1,
                                                                            const Vector3d<Navigation>& angularVelocity_ie_n__t1,
                                                                            const Vector3d<Navigation>& angularVelocity_en_n__t1,
                                                                            const Quaterniond<Navigation, Body>& quaternion_nb__t1,
                                                                            const Quaterniond<Navigation, Body>& quaternion_nb__t2);

/// @brief Integrates the accelerations and calculates the new velocity v_e with Runge Kutta of 3rd Order
/// @param[in] timeDifferenceSec__t0 Δtₖ Time difference in [seconds]. This epoch to previous epoch
/// @param[in] timeDifferenceSec__t1 Δtₖ₋₁ Time difference in [seconds]. Previous epoch to twice previous epoch
/// @param[in] acceleration_p__t0 a_p (tₖ) Acceleration in [m/s^2], in platform coordinates, at the time tₖ
/// @param[in] acceleration_p__t1 a_p (tₖ₋₁) Acceleration in [m/s^2], in platform coordinates, at the time tₖ₋₁
/// @param[in] velocity_e__t2 v_e (tₖ₋₂) Velocity in [m/s], in earth coordinates, at the time tₖ₋₂
/// @param[in] position_e__t2 x_e (tₖ₋₂) Position in [m/s], in earth coordinates, at the time tₖ₋₂
/// @param[in] gravity_e g_e Gravity vector in [m/s^2], in earth coordinates
/// @param[in] quaternion_ep__t0 q (tₖ) Quaternion, from platform to earth coordinates, at the time tₖ
/// @param[in] quaternion_ep__t1 q (tₖ₋₁) Quaternion, from platform to earth coordinates, at the time tₖ₋₁
/// @param[in] quaternion_ep__t2 q (tₖ₋₂) Quaternion, from platform to earth coordinates, at the time tₖ₋₂
/// @return The updated velocity v_e
///
/// @note See C. Jekeli (2001) - Inertial Navigation Systems with Geodetic Applications (Chapter 4.2.4.1.2)
///       See T. Hobiger - Inertialnavigation (Lecture Slides Chapter 8)
[[nodiscard]] Vector3d<Earth> updateVelocity_e_RungeKutta3(const long double& timeDifferenceSec__t0,
                                                           const long double& timeDifferenceSec__t1,
                                                           const Vector3d<Platform>& acceleration_p__t0,
                                                           const Vector3d<Platform>& acceleration_p__t1,
                                                           const Vector3d<Earth>& velocity_e__t2,
                                                           const Vector3d<Earth>& position_e__t2,
                                                           const Vector3d<Earth>& gravity_e,
                                                           const Quaterniond<Earth, Platform>& quaternion_ep__t0,
                                                           const Quaterniond<Earth, Platform>& quaternion_ep__t1,
                                                           const Quaterniond<Earth, Platform>& quaternion_ep__t2);

/// @brief Integrates the accelerations and calculates the new velocity v_n with Runge Kutta of 3rd Order
/// @param[in] timeDifferenceSec__t0 Δtₖ Time difference in [seconds]. This epoch to previous epoch
/// @param[in] timeDifferenceSec__t1 Δtₖ₋₁ Time difference in [seconds]. Previous epoch to twice previous epoch
/// @param[in] acceleration_b__t0 a_p (tₖ) Acceleration in [m/s^2], in body coordinates, at the time tₖ
/// @param[in] acceleration_b__t1 a_p (tₖ₋₁) Acceleration in [m/s^2], in body coordinates, at the time tₖ₋₁
/// @param[in] velocity_n__t1 v_n (tₖ₋₁) Velocity in [m/s], in navigation coordinates, at the time tₖ₋₁
/// @param[in] velocity_n__t2 v_n (tₖ₋₂) Velocity in [m/s], in navigation coordinates, at the time tₖ₋₂
/// @param[in] gravity_n__t1 g_n (tₖ₋₁) Gravity vector in [m/s^2], in navigation coordinates, at the time tₖ₋₁
/// @param[in] angularVelocity_ie_n__t1 ω_ie_n (tₖ₋₁) Nominal mean angular velocity of the Earth in [rad/s], in navigation coordinates, at the time tₖ₋₁
/// @param[in] angularVelocity_en_n__t1 ω_ie_n (tₖ₋₁) Transport Rate in [rad/s], in navigation coordinates, at the time tₖ₋₁
/// @param[in] quaternion_nb__t0 q (tₖ) Quaternion, from body to navigation coordinates, at the time tₖ
/// @param[in] quaternion_nb__t1 q (tₖ₋₁) Quaternion, from body to navigation coordinates, at the time tₖ₋₁
/// @param[in] quaternion_nb__t2 q (tₖ₋₂) Quaternion, from body to navigation coordinates, at the time tₖ₋₂
/// @return The updated velocity v_n
///
/// @note See S. Gleason (2009) - GNSS Applications and Methods (Chapter 6.2.3.2)
[[nodiscard]] Vector3d<Navigation> updateVelocity_n_RungeKutta3(const long double& timeDifferenceSec__t0,
                                                                const long double& timeDifferenceSec__t1,
                                                                const Vector3d<Body>& acceleration_b__t0,
                                                                const Vector3d<Body>& acceleration_b__t1,
                                                                const Vector3d<Navigation>& velocity_n__t1,
                                                                const Vector3d<Navigation>& velocity_n__t2,
                                                                const Vector3d<Navigation>& gravity_n__t1,
                                                                const Vector3d<Navigation>& angularVelocity_ie_n__t1,
                                                                const Vector3d<Navigation>& angularVelocity_en_n__t1,
                                                                const Quaterniond<Navigation, Body>& quaternion_nb__t0,
                                                                const Quaterniond<Navigation, Body>& quaternion_nb__t1,
                                                                const Quaterniond<Navigation, Body>& quaternion_nb__t2);

/// @brief Calculates the new position x_e in earth frame
/// @param[in] timeDifferenceSec__t0 Δtₖ Time difference in [seconds]. This epoch to previous epoch
/// @param[in] position_e__t1 x_e (tₖ₋₁) Position in [m/s], in earth coordinates, at the time tₖ₋₁
/// @param[in] velocity_e__t1 v_e (tₖ₋₁) Velocity in [m/s], in earth coordinates, at the time tₖ₋₁
/// @return x_e (tₖ) Position in [m/s], in earth coordinates, at the time tₖ
///
/// @note See C. Jekeli (2001) - Inertial Navigation Systems with Geodetic Applications (Chapter 4.2.4.1.2)
///       See T. Hobiger - Inertialnavigation (Lecture Slides Chapter 8)
[[nodiscard]] Vector3d<Earth> updatePosition_e(const long double& timeDifferenceSec__t0,
                                               const Vector3d<Earth>& position_e__t1,
                                               const Vector3d<Earth>& velocity_e__t1);

/// @brief Calculates the new position [𝜙, λ, h]
/// @param[in] timeDifferenceSec__t0 Δtₖ Time difference in [seconds]. This epoch to previous epoch
/// @param[in] latLonAlt__t1 [𝜙, λ, h] (tₖ₋₁) Latitude, Longitude and altitude in [rad, rad, m] at the time tₖ₋₁
/// @param[in] velocity_n__t1 v_n (tₖ₋₁) Velocity in [m/s], in navigation coordinates, at the time tₖ₋₁
/// @param[in] R_N North/South (meridian) earth radius [m]
/// @param[in] R_E East/West (prime vertical) earth radius [m]
/// @return [𝜙, λ, h] (tₖ) Latitude, Longitude and altitude in [rad, rad, m] at the time tₖ
///
/// @note See S. Gleason (2009) - GNSS Applications and Methods (Chapter 6.2.3.3)
[[nodiscard]] Vector3d<LLA> updatePosition_lla(const long double& timeDifferenceSec__t0,
                                               const Vector3d<LLA>& latLonAlt__t1,
                                               const Vector3d<Navigation>& velocity_n__t1,
                                               const double& R_N,
                                               const double& R_E);

/// @brief Calculates the new position [x_n, x_e, x_d]
/// @param[in] timeDifferenceSec__t0 Δtₖ Time difference in [seconds]. This epoch to previous epoch
/// @param[in] position_n__t1 [x_n, x_e, x_d] (tₖ₋₁) Position NED in [m] at the time tₖ₋₁
/// @param[in] velocity_n__t1 v_n (tₖ₋₁) Velocity in [m/s], in navigation coordinates, at the time tₖ₋₁
/// @return [x_n, x_e, x_d] (tₖ) Position NED in [m] at the time tₖ
[[nodiscard]] Vector3d<Navigation> updatePosition_n(const long double& timeDifferenceSec__t0,
                                                    const Vector3d<Navigation>& position_n__t1,
                                                    const Vector3d<Navigation>& velocity_n__t1);

/// @brief Calculates the North/South (meridian) earth radius
/// @param[in] a Semi-major axis
/// @param[in] e_squared Square of the first eccentricity of the ellipsoid
/// @param[in] latitude 𝜙 Latitude in [rad]
/// @return North/South (meridian) earth radius [m]
[[nodiscard]] double earthRadius_N(const double& a, const double& e_squared, const double& latitude);

/// @brief Calculates the East/West (prime vertical) earth radius
/// @param[in] a Semi-major axis
/// @param[in] e_squared Square of the first eccentricity of the ellipsoid
/// @param[in] latitude 𝜙 Latitude in [rad]
/// @return East/West (prime vertical) earth radius [m]
[[nodiscard]] double earthRadius_E(const double& a, const double& e_squared, const double& latitude);

/// @brief Calculates the transport rate ω_en_n
/// @param[in] latLonAlt__t1 [𝜙, λ, h] (tₖ₋₁) Latitude, Longitude and altitude in [rad, rad, m] at the time tₖ₋₁
/// @param[in] velocity_n__t1 v_n (tₖ₋₁) Velocity in [m/s], in navigation coordinates, at the time tₖ₋₁
/// @param[in] R_N North/South (meridian) earth radius [m]
/// @param[in] R_E East/West (prime vertical) earth radius [m]
/// @return ω_en_n (tₖ₋₁) Transport Rate, rotation rate of the Earth frame relative to the navigation frame, in navigation coordinates
[[nodiscard]] Vector3d<Navigation> transportRate(const Vector3d<LLA>& latLonAlt__t1,
                                                 const Vector3d<Navigation>& velocity_n__t1,
                                                 const double& R_N,
                                                 const double& R_E);

} // namespace NAV
