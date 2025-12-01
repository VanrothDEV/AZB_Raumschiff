//! Simulations-Modul: 6-DOF Mission Simulation
//!
//! Führt die gesamte Mondmission durch:
//! - Initialisierung auf Erdoberfläche
//! - Aufstieg, Transfer, Orbit, Landung
//! - Echtzeit-Telemetrie

use crate::physics::{
    self, SpacecraftState, EARTH_MOON_DISTANCE,
};
use crate::gnc::{GuidanceComputer, KalmanFilter, MissionPhase, add_sensor_noise};
use crate::fdir::FDIRManager;
use crate::telemetry::TelemetryLogger;
use nalgebra::{Vector3, Vector6};

/// Simulationsparameter
pub struct SimConfig {
    /// Zeitschritt [s]
    pub dt: f64,
    /// Maximale Simulationszeit [s]
    pub max_time: f64,
    /// Spezifischer Impuls [s]
    pub isp: f64,
    /// Maximaler Schub [N]
    pub max_thrust: f64,
    /// Startmasse [kg]
    pub initial_mass: f64,
    /// Trockenmasse [kg]
    pub dry_mass: f64,
    /// Telemetrie-Intervall [s]
    pub telemetry_interval: f64,
}

impl Default for SimConfig {
    fn default() -> Self {
        Self {
            dt: 1.0,                    // 1 Sekunde Zeitschritt
            max_time: 5.0 * 24.0 * 3600.0, // 5 Tage max (typische Mondmission)
            isp: 450.0,                 // Guter chemischer Antrieb (RL-10 Niveau)
            max_thrust: 500_000.0,      // 500 kN (starke obere Stufe)
            initial_mass: 250_000.0,    // 250 Tonnen (mehr Treibstoff)
            dry_mass: 15_000.0,         // 15 Tonnen Trockenmasse
            telemetry_interval: 60.0,   // Alle 60 Sekunden
        }
    }
}

/// Simulationsergebnis
pub struct SimResult {
    pub success: bool,
    pub final_state: SpacecraftState,
    pub mission_time: f64,
    pub fuel_used: f64,
    pub telemetry: TelemetryLogger,
}

/// Hauptsimulation
pub struct MoonMissionSim {
    pub config: SimConfig,
    pub state: SpacecraftState,
    pub earth_pos: Vector3<f64>,
    pub moon_pos: Vector3<f64>,
    pub guidance: GuidanceComputer,
    pub kalman: KalmanFilter,
    pub fdir: FDIRManager,
    pub telemetry: TelemetryLogger,
}

impl MoonMissionSim {
    pub fn new(config: SimConfig) -> Self {
        // Erde im Ursprung
        let earth_pos = Vector3::zeros();

        // Mond auf X-Achse
        let moon_pos = Vector3::new(EARTH_MOON_DISTANCE, 0.0, 0.0);

        // Raumschiff startet in niedrigem Erdorbit (LEO, 400 km Höhe)
        // Für eine direkte Trans-Lunar-Injection (TLI) wird während des TLI-Burns
        // in Flugrichtung beschleunigt. Die optimale Startposition ist dort,
        // wo die Tangentialgeschwindigkeit nach dem Burn zum Mond zeigt.
        let orbit_altitude: f64 = 400_000.0; // 400 km
        let orbit_radius: f64 = 6.371e6 + orbit_altitude;
        let orbital_velocity: f64 = (6.67430e-11_f64 * 5.972e24_f64 / orbit_radius).sqrt();
        
        // Startposition: Im Orbit, Geschwindigkeit zeigt zum Mond (+X)
        // Position bei (0, -R, 0), Geschwindigkeit bei (+v, 0, 0)
        let initial_pos = Vector3::new(0.0, -orbit_radius, 0.0);
        let initial_vel = Vector3::new(orbital_velocity, 0.0, 0.0);

        let state = SpacecraftState::new(initial_pos, initial_vel, config.initial_mass);

        // Ziel: Mondoberfläche
        let moon_surface = moon_pos - Vector3::new(1.737e6, 0.0, 0.0);
        let guidance = GuidanceComputer::new(moon_surface, config.max_thrust);

        // Kalman-Filter initialisieren
        let kalman_state = Vector6::new(
            initial_pos.x,
            initial_pos.y,
            initial_pos.z,
            initial_vel.x,
            initial_vel.y,
            initial_vel.z,
        );
        let kalman = KalmanFilter::new(kalman_state);

        let fdir = FDIRManager::new();
        let telemetry = TelemetryLogger::new();

        Self {
            config,
            state,
            earth_pos,
            moon_pos,
            guidance,
            kalman,
            fdir,
            telemetry,
        }
    }

    /// Führt die komplette Mission durch
    pub fn run(&mut self) -> SimResult {
        let orbit_altitude = self.state.position.norm() - 6.371e6;
        let orbit_velocity = self.state.velocity.norm();
        
        println!("🚀 AZB_Raumschiff Mission Start!");
        println!("   Ziel: Mondlandung");
        println!("   Startposition: LEO ({:.0} km Höhe, {:.0} m/s)", orbit_altitude / 1000.0, orbit_velocity);
        println!("   Startmasse: {:.0} kg", self.config.initial_mass);
        println!("   Max. Schub: {:.0} kN", self.config.max_thrust / 1000.0);
        println!();

        let initial_mass = self.config.initial_mass;
        let mut last_telemetry = 0.0;
        let mut iteration = 0;

        while self.state.time < self.config.max_time {
            // FDIR-Zyklus
            self.fdir.run_cycle();
            if !self.fdir.is_operational() {
                println!("❌ Mission aborted: System critical failure");
                break;
            }

            // Erdkollisionserkennung
            let earth_altitude = self.state.position.norm() - 6.371e6;
            if earth_altitude < -100.0 {
                println!("💥 Mission failed: Collision with Earth!");
                break;
            }

            // Schub berechnen (Guidance)
            let thrust = self.guidance.compute_thrust(
                &self.state.position,
                &self.state.velocity,
                &self.moon_pos,
            );

            // Physik-Integration (RK4)
            physics::integrate_rk4(
                &mut self.state,
                &self.earth_pos,
                &self.moon_pos,
                &thrust,
                self.config.isp,
                self.config.dt,
            );

            // Kalman-Filter Update
            self.kalman.predict(self.config.dt);
            let noisy_pos = add_sensor_noise(&self.state.position, 100.0);
            self.kalman.update(&noisy_pos);

            // Telemetrie
            if self.state.time - last_telemetry >= self.config.telemetry_interval {
                self.log_telemetry();
                last_telemetry = self.state.time;
            }

            // Status-Ausgabe (alle 1000 Iterationen)
            if iteration % 1000 == 0 {
                self.print_status();
            }

            // FDIR nominal melden
            self.fdir.report_nominal();

            // Erfolgscheck
            if self.guidance.phase == MissionPhase::Landed {
                println!();
                println!("✅ MISSION SUCCESS!");
                return SimResult {
                    success: true,
                    final_state: self.state.clone(),
                    mission_time: self.state.time,
                    fuel_used: initial_mass - self.state.mass,
                    telemetry: std::mem::take(&mut self.telemetry),
                };
            }

            // Treibstoff-Check
            if self.state.mass <= self.config.dry_mass {
                println!("⛽ Mission failed: Out of fuel!");
                break;
            }

            iteration += 1;
        }

        SimResult {
            success: false,
            final_state: self.state.clone(),
            mission_time: self.state.time,
            fuel_used: initial_mass - self.state.mass,
            telemetry: std::mem::take(&mut self.telemetry),
        }
    }

    fn log_telemetry(&mut self) {
        let pos = self.state.position;
        let vel = self.state.velocity;
        self.telemetry
            .log_navigation([pos.x, pos.y, pos.z], [vel.x, vel.y, vel.z]);

        let fuel_percent =
            (self.state.mass - self.config.dry_mass) / (self.config.initial_mass - self.config.dry_mass) * 100.0;

        self.telemetry.log_status(
            self.guidance.phase as u8,
            fuel_percent as f32,
            if self.fdir.is_operational() { 100 } else { 0 },
        );
    }

    fn print_status(&self) {
        let distance_earth = self.state.position.norm();
        let distance_moon = (self.moon_pos - self.state.position).norm();
        let speed = self.state.velocity.norm();
        let fuel_percent =
            (self.state.mass - self.config.dry_mass) / (self.config.initial_mass - self.config.dry_mass) * 100.0;

        println!(
            "T+{:>8.0}s | Phase: {:?} | Alt Earth: {:>10.0}km | Dist Moon: {:>10.0}km | Speed: {:>8.1}m/s | Fuel: {:>5.1}%",
            self.state.time,
            self.guidance.phase,
            (distance_earth - 6.371e6) / 1000.0,
            distance_moon / 1000.0,
            speed,
            fuel_percent
        );
    }
}

/// Schnellstart-Funktion
pub fn run_moon_mission() -> SimResult {
    let config = SimConfig::default();
    let mut sim = MoonMissionSim::new(config);
    sim.run()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_sim_initialization() {
        let config = SimConfig::default();
        let sim = MoonMissionSim::new(config);

        assert!(sim.state.mass > 0.0);
        assert!(sim.state.position.norm() > 0.0);
    }

    #[test]
    fn test_short_simulation() {
        let config = SimConfig {
            dt: 10.0,
            max_time: 100.0, // Nur 100 Sekunden
            ..Default::default()
        };
        let mut sim = MoonMissionSim::new(config);
        let result = sim.run();

        // Sollte noch nicht gelandet sein
        assert!(!result.success);
        assert!(result.mission_time >= 100.0);
    }
}
