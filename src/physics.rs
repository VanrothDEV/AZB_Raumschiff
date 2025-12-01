//! Physik-Modul: Translationsdynamik, Gravitation, Massenstrom
//!
//! Formeln aus README.md:
//! - F = m * a
//! - r(t) = r0 + v0*t + 0.5*a*t²
//! - ṁ = -T / (Isp * g0)

use nalgebra::Vector3;

/// Gravitationskonstante [m³/(kg·s²)]
pub const G: f64 = 6.67430e-11;

/// Erdmasse [kg]
pub const M_EARTH: f64 = 5.972e24;

/// Mondmasse [kg]
pub const M_MOON: f64 = 7.342e22;

/// Erdradius [m]
pub const R_EARTH: f64 = 6.371e6;

/// Mondradius [m]
pub const R_MOON: f64 = 1.737e6;

/// Erdbeschleunigung [m/s²]
pub const G0: f64 = 9.80665;

/// Erde-Mond Distanz [m] (mittlere)
pub const EARTH_MOON_DISTANCE: f64 = 384_400_000.0;

/// Zustand des Raumschiffs
#[derive(Debug, Clone)]
pub struct SpacecraftState {
    /// Position [m] im inertialen Referenzsystem
    pub position: Vector3<f64>,
    /// Geschwindigkeit [m/s]
    pub velocity: Vector3<f64>,
    /// Masse [kg]
    pub mass: f64,
    /// Zeit seit Start [s]
    pub time: f64,
}

impl SpacecraftState {
    pub fn new(position: Vector3<f64>, velocity: Vector3<f64>, mass: f64) -> Self {
        Self {
            position,
            velocity,
            mass,
            time: 0.0,
        }
    }
}

/// Berechnet Gravitationskraft zwischen zwei Körpern
/// F = G * m1 * m2 / r² (Richtung: von m1 zu m2)
pub fn gravitational_force(
    pos1: &Vector3<f64>,
    mass1: f64,
    pos2: &Vector3<f64>,
    mass2: f64,
) -> Vector3<f64> {
    let r = pos2 - pos1;
    let distance = r.norm();
    if distance < 1.0 {
        return Vector3::zeros(); // Singularität vermeiden
    }
    let force_magnitude = G * mass1 * mass2 / (distance * distance);
    r.normalize() * force_magnitude
}

/// Berechnet Gravitationsbeschleunigung auf das Raumschiff
/// von Erde und Mond (vereinfachtes 2-Körper-Problem)
pub fn gravity_acceleration(
    spacecraft_pos: &Vector3<f64>,
    earth_pos: &Vector3<f64>,
    moon_pos: &Vector3<f64>,
) -> Vector3<f64> {
    // Richtung zu Erde
    let r_earth = earth_pos - spacecraft_pos;
    let d_earth = r_earth.norm();
    let a_earth = if d_earth > 1.0 {
        r_earth.normalize() * (G * M_EARTH / (d_earth * d_earth))
    } else {
        Vector3::zeros()
    };

    // Richtung zu Mond
    let r_moon = moon_pos - spacecraft_pos;
    let d_moon = r_moon.norm();
    let a_moon = if d_moon > 1.0 {
        r_moon.normalize() * (G * M_MOON / (d_moon * d_moon))
    } else {
        Vector3::zeros()
    };

    a_earth + a_moon
}

/// Schubkraft-Beschleunigung
/// a = F / m
pub fn thrust_acceleration(thrust: &Vector3<f64>, mass: f64) -> Vector3<f64> {
    if mass > 0.0 {
        thrust / mass
    } else {
        Vector3::zeros()
    }
}

/// Treibstoff-Massenstrom [kg/s]
/// ṁ = -T / (Isp * g0)
pub fn propellant_mass_flow(thrust_magnitude: f64, isp: f64) -> f64 {
    if isp > 0.0 {
        thrust_magnitude / (isp * G0)
    } else {
        0.0
    }
}

/// Integriert Zustand um dt (Euler-Verfahren, später RK4)
pub fn integrate_state(
    state: &mut SpacecraftState,
    acceleration: &Vector3<f64>,
    mass_flow: f64,
    dt: f64,
) {
    // Geschwindigkeit aktualisieren: v = v + a*dt
    state.velocity += acceleration * dt;

    // Position aktualisieren: r = r + v*dt + 0.5*a*dt²
    state.position += state.velocity * dt + 0.5 * acceleration * dt * dt;

    // Masse aktualisieren
    state.mass -= mass_flow * dt;
    if state.mass < 100.0 {
        state.mass = 100.0; // Trockenmasse minimum
    }

    state.time += dt;
}

/// Runge-Kutta 4. Ordnung Integration für höhere Genauigkeit
pub fn integrate_rk4(
    state: &mut SpacecraftState,
    earth_pos: &Vector3<f64>,
    moon_pos: &Vector3<f64>,
    thrust: &Vector3<f64>,
    isp: f64,
    dt: f64,
) {
    let mass_flow = propellant_mass_flow(thrust.norm(), isp);

    // k1
    let a1 = gravity_acceleration(&state.position, earth_pos, moon_pos)
        + thrust_acceleration(thrust, state.mass);
    let v1 = state.velocity;

    // k2
    let pos2 = state.position + v1 * (dt / 2.0);
    let vel2 = state.velocity + a1 * (dt / 2.0);
    let a2 = gravity_acceleration(&pos2, earth_pos, moon_pos)
        + thrust_acceleration(thrust, state.mass - mass_flow * dt / 2.0);

    // k3
    let pos3 = state.position + vel2 * (dt / 2.0);
    let vel3 = state.velocity + a2 * (dt / 2.0);
    let a3 = gravity_acceleration(&pos3, earth_pos, moon_pos)
        + thrust_acceleration(thrust, state.mass - mass_flow * dt / 2.0);

    // k4
    let pos4 = state.position + vel3 * dt;
    let vel4 = state.velocity + a3 * dt;
    let a4 = gravity_acceleration(&pos4, earth_pos, moon_pos)
        + thrust_acceleration(thrust, state.mass - mass_flow * dt);

    // Kombinieren
    state.position += (v1 + 2.0 * vel2 + 2.0 * vel3 + vel4) * (dt / 6.0);
    state.velocity += (a1 + 2.0 * a2 + 2.0 * a3 + a4) * (dt / 6.0);
    state.mass -= mass_flow * dt;
    if state.mass < 100.0 {
        state.mass = 100.0;
    }
    state.time += dt;
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_gravity_earth_surface() {
        let spacecraft = Vector3::new(6.371e6, 0.0, 0.0); // Erdoberfläche
        let earth = Vector3::zeros();
        let moon = Vector3::new(EARTH_MOON_DISTANCE, 0.0, 0.0);

        let a = gravity_acceleration(&spacecraft, &earth, &moon);
        // Sollte ca. 9.8 m/s² Richtung Erde sein
        assert!((a.norm() - 9.8).abs() < 0.5);
    }

    #[test]
    fn test_mass_flow() {
        let thrust = 100_000.0; // 100 kN
        let isp = 300.0; // s
        let mdot = propellant_mass_flow(thrust, isp);
        // ṁ = 100000 / (300 * 9.80665) ≈ 34 kg/s
        assert!((mdot - 34.0).abs() < 1.0);
    }
}
