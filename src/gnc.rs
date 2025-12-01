//! GNC-Modul: Guidance, Navigation & Control
//!
//! Enthält:
//! - Kalman-Filter für Zustandsschätzung
//! - Quaternion-basierte Lageregelung
//! - Einfache Guidance-Logik für Mondlandung

use nalgebra::{Matrix3, Matrix6, Vector3, Vector6, UnitQuaternion};
use rand::Rng;

/// Kalman-Filter Zustand (Position + Geschwindigkeit)
#[derive(Debug, Clone)]
pub struct KalmanFilter {
    /// Geschätzter Zustand [x, y, z, vx, vy, vz]
    pub state: Vector6<f64>,
    /// Kovarianzmatrix P
    pub covariance: Matrix6<f64>,
    /// Prozessrauschen Q
    pub process_noise: Matrix6<f64>,
    /// Messrauschen R (nur Position messbar)
    pub measurement_noise: Matrix3<f64>,
}

impl KalmanFilter {
    pub fn new(initial_state: Vector6<f64>) -> Self {
        Self {
            state: initial_state,
            covariance: Matrix6::identity() * 1000.0,
            process_noise: Matrix6::identity() * 0.1,
            measurement_noise: Matrix3::identity() * 10.0,
        }
    }

    /// Predict-Schritt: x_k|k-1 = F * x_k-1
    pub fn predict(&mut self, dt: f64) {
        // Zustandsübergangsmatrix F (konstante Geschwindigkeit)
        let mut f = Matrix6::identity();
        f[(0, 3)] = dt;
        f[(1, 4)] = dt;
        f[(2, 5)] = dt;

        // Prädiktion
        self.state = f * self.state;
        self.covariance = f * self.covariance * f.transpose() + self.process_noise * dt;
    }

    /// Update-Schritt mit Positionsmessung
    /// x_k|k = x_k|k-1 + K * (z - H * x_k|k-1)
    /// K = P * H^T * (H * P * H^T + R)^-1
    pub fn update(&mut self, measurement: &Vector3<f64>) {
        // Beobachtungsmatrix H (nur Position)
        let mut h = nalgebra::Matrix3x6::zeros();
        h[(0, 0)] = 1.0;
        h[(1, 1)] = 1.0;
        h[(2, 2)] = 1.0;

        // Innovation
        let predicted_measurement = h * self.state;
        let innovation = measurement - predicted_measurement;

        // Kalman-Gain: K = P * H^T * (H * P * H^T + R)^-1
        let s = h * self.covariance * h.transpose() + self.measurement_noise;
        if let Some(s_inv) = s.try_inverse() {
            let k = self.covariance * h.transpose() * s_inv;

            // Zustand aktualisieren
            self.state += k * innovation;

            // Kovarianz aktualisieren
            let i = Matrix6::identity();
            self.covariance = (i - k * h) * self.covariance;
        }
    }

    /// Gibt geschätzte Position zurück
    pub fn estimated_position(&self) -> Vector3<f64> {
        Vector3::new(self.state[0], self.state[1], self.state[2])
    }

    /// Gibt geschätzte Geschwindigkeit zurück
    pub fn estimated_velocity(&self) -> Vector3<f64> {
        Vector3::new(self.state[3], self.state[4], self.state[5])
    }
}

/// Lage (Attitude) des Raumschiffs
#[derive(Debug, Clone)]
pub struct AttitudeController {
    /// Aktuelle Orientierung als Quaternion
    pub orientation: UnitQuaternion<f64>,
    /// Winkelgeschwindigkeit [rad/s]
    pub angular_velocity: Vector3<f64>,
    /// Ziel-Orientierung
    pub target_orientation: UnitQuaternion<f64>,
    /// Regelparameter (P-Anteil)
    pub kp: f64,
    /// Regelparameter (D-Anteil)
    pub kd: f64,
}

impl AttitudeController {
    pub fn new() -> Self {
        Self {
            orientation: UnitQuaternion::identity(),
            angular_velocity: Vector3::zeros(),
            target_orientation: UnitQuaternion::identity(),
            kp: 2.0,
            kd: 1.0,
        }
    }

    /// Setzt Ziel-Orientierung basierend auf gewünschter Schubrichtung
    pub fn point_towards(&mut self, direction: &Vector3<f64>) {
        if direction.norm() > 1e-6 {
            let forward = Vector3::new(0.0, 0.0, 1.0);
            self.target_orientation =
                UnitQuaternion::rotation_between(&forward, &direction.normalize())
                    .unwrap_or(UnitQuaternion::identity());
        }
    }

    /// Berechnet benötigtes Drehmoment (PD-Regler)
    /// τ = Kp * θ_error - Kd * ω
    pub fn compute_torque(&self) -> Vector3<f64> {
        // Quaternion-Fehler
        let q_error = self.target_orientation * self.orientation.inverse();
        let axis_angle = q_error.scaled_axis();

        // PD-Regelgesetz
        self.kp * axis_angle - self.kd * self.angular_velocity
    }

    /// Aktualisiert Orientierung basierend auf Drehmoment
    /// q̇ = 0.5 * Ω(ω) * q
    pub fn update(&mut self, torque: &Vector3<f64>, inertia: f64, dt: f64) {
        // Winkelbeschleunigung: α = τ / I
        let angular_accel = torque / inertia;

        // Winkelgeschwindigkeit aktualisieren
        self.angular_velocity += angular_accel * dt;

        // Quaternion-Kinematik: q̇ = 0.5 * ω * q
        let omega_quat = UnitQuaternion::from_scaled_axis(self.angular_velocity * dt);
        self.orientation = omega_quat * self.orientation;
    }
}

impl Default for AttitudeController {
    fn default() -> Self {
        Self::new()
    }
}

/// Guidance-System für Mondlandung
#[derive(Debug, Clone)]
pub struct GuidanceComputer {
    /// Zielposition (Mondoberfläche)
    pub target_position: Vector3<f64>,
    /// Zielgeschwindigkeit (sanfte Landung)
    pub target_velocity: Vector3<f64>,
    /// Maximaler Schub [N]
    pub max_thrust: f64,
    /// Aktueller Missionszustand
    pub phase: MissionPhase,
    /// TLI abgeschlossen
    pub tli_complete: bool,
    /// LOI abgeschlossen  
    pub loi_complete: bool,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum MissionPhase {
    /// Aufstieg von der Erde
    Ascent,
    /// Transferbahn zum Mond
    TransLunarInjection,
    /// Mondorbit-Eintritt
    LunarOrbitInsertion,
    /// Abstieg zur Oberfläche
    Descent,
    /// Gelandet
    Landed,
}

impl GuidanceComputer {
    pub fn new(moon_surface: Vector3<f64>, max_thrust: f64) -> Self {
        Self {
            target_position: moon_surface,
            target_velocity: Vector3::zeros(),
            max_thrust,
            phase: MissionPhase::TransLunarInjection, // Starte direkt mit TLI (bereits im Orbit)
            tli_complete: false,
            loi_complete: false,
        }
    }

    /// Berechnet Schubvektor basierend auf aktuellem Zustand
    pub fn compute_thrust(
        &mut self,
        position: &Vector3<f64>,
        velocity: &Vector3<f64>,
        moon_pos: &Vector3<f64>,
    ) -> Vector3<f64> {
        let distance_to_moon = (moon_pos - position).norm();
        let distance_to_earth = position.norm();
        let speed = velocity.norm();
        let altitude_earth = distance_to_earth - 6.371e6;

        // Phasenwechsel-Logik
        self.update_phase(distance_to_moon, speed, distance_to_earth);

        match self.phase {
            MissionPhase::Ascent => Vector3::zeros(), // Nicht verwendet
            
            MissionPhase::TransLunarInjection => {
                // TLI: Kurzer Burn bis ~10.8 km/s, dann Coast
                if !self.tli_complete && speed < 10_800.0 {
                    velocity.normalize() * self.max_thrust
                } else {
                    if !self.tli_complete {
                        self.tli_complete = true;
                        println!("🔥 TLI Burn complete! Coasting to Moon... (v: {:.0}m/s)", speed);
                    }
                    Vector3::zeros() // COAST - kein Schub!
                }
            }
            
            MissionPhase::LunarOrbitInsertion => {
                // LOI: Bremsen wenn nahe am Mond
                if !self.loi_complete && speed > 800.0 {
                    -velocity.normalize() * self.max_thrust * 0.5
                } else {
                    if !self.loi_complete && speed <= 800.0 {
                        self.loi_complete = true;
                        println!("🔥 LOI Burn complete! In lunar orbit (v: {:.0}m/s)", speed);
                    }
                    Vector3::zeros()
                }
            }
            
            MissionPhase::Descent => {
                let alt_moon = distance_to_moon - 1.737e6;
                
                // Sanfte Landung: Geschwindigkeit proportional zur Höhe
                let target_speed = if alt_moon > 50_000.0 {
                    300.0
                } else if alt_moon > 5_000.0 {
                    100.0
                } else if alt_moon > 500.0 {
                    30.0
                } else {
                    5.0
                };
                
                if speed > target_speed {
                    -velocity.normalize() * self.max_thrust * 0.8
                } else {
                    Vector3::zeros()
                }
            }
            
            MissionPhase::Landed => Vector3::zeros(),
        }
    }

    fn update_phase(&mut self, distance_to_moon: f64, speed: f64, distance_to_earth: f64) {
        let altitude_earth = distance_to_earth - 6.371e6;
        
        match self.phase {
            MissionPhase::Ascent => {
                // LEO erreicht: 185km+, 7.7+ km/s
                if altitude_earth > 185_000.0 && speed >= 7_700.0 {
                    self.phase = MissionPhase::TransLunarInjection;
                    println!("🚀 Phase: Trans-Lunar Injection (alt: {:.0}km, v: {:.0}m/s)", 
                             altitude_earth/1000.0, speed);
                }
            }
            MissionPhase::TransLunarInjection => {
                // Nahe Mond und TLI abgeschlossen
                if distance_to_moon < 66_000_000.0 {
                    self.phase = MissionPhase::LunarOrbitInsertion;
                    println!("🌙 Phase: Lunar Orbit Insertion (dist: {:.0}km, v: {:.0}m/s)", 
                             distance_to_moon/1000.0, speed);
                }
            }
            MissionPhase::LunarOrbitInsertion => {
                // Mondorbit erreicht: <2000km, <1.7 km/s
                let alt_moon = distance_to_moon - 1.737e6;
                if alt_moon < 200_000.0 && speed < 1_700.0 {
                    self.phase = MissionPhase::Descent;
                    println!("⬇️ Phase: Descent (alt: {:.0}km, v: {:.0}m/s)", 
                             alt_moon/1000.0, speed);
                }
            }
            MissionPhase::Descent => {
                // Touchdown
                let altitude_moon = distance_to_moon - 1.737e6;
                if altitude_moon < 10.0 && speed < 3.0 {
                    self.phase = MissionPhase::Landed;
                    println!("🎉 LANDED ON THE MOON! (alt: {:.1}m, v: {:.1}m/s)", 
                             altitude_moon, speed);
                }
            }
            MissionPhase::Landed => {}
        }
    }
}

/// Fügt Sensorrauschen hinzu (für realistische Simulation)
pub fn add_sensor_noise(value: &Vector3<f64>, stddev: f64) -> Vector3<f64> {
    let mut rng = rand::thread_rng();
    Vector3::new(
        value.x + rng.gen::<f64>() * stddev - stddev / 2.0,
        value.y + rng.gen::<f64>() * stddev - stddev / 2.0,
        value.z + rng.gen::<f64>() * stddev - stddev / 2.0,
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_kalman_filter() {
        let initial = Vector6::new(0.0, 0.0, 0.0, 100.0, 0.0, 0.0);
        let mut kf = KalmanFilter::new(initial);

        kf.predict(1.0);
        assert!((kf.state[0] - 100.0).abs() < 1.0); // x = vx * t

        let measurement = Vector3::new(105.0, 0.0, 0.0);
        kf.update(&measurement);
        // Position sollte zwischen Prädiktion und Messung liegen
        assert!(kf.state[0] > 100.0 && kf.state[0] < 105.0);
    }

    #[test]
    fn test_attitude_controller() {
        let mut ctrl = AttitudeController::new();
        ctrl.point_towards(&Vector3::new(1.0, 0.0, 0.0));

        let torque = ctrl.compute_torque();
        assert!(torque.norm() > 0.0); // Sollte Drehmoment erzeugen
    }
}
