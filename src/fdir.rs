//! FDIR-Modul: Fault Detection, Isolation & Recovery
//!
//! Implementiert:
//! - Triple Modular Redundancy (TMR)
//! - Systemüberwachung (Watchdog)
//! - Graceful Degradation

use std::time::{Duration, Instant};

/// Systemstatus
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum SystemStatus {
    Nominal,
    Warning,
    Fault,
    Critical,
}

/// Einzelnes Subsystem mit Redundanz
#[derive(Debug, Clone)]
pub struct RedundantSubsystem<T: Clone + PartialEq> {
    pub name: String,
    /// Drei redundante Werte (TMR)
    pub values: [Option<T>; 3],
    pub status: SystemStatus,
}

impl<T: Clone + PartialEq> RedundantSubsystem<T> {
    pub fn new(name: &str) -> Self {
        Self {
            name: name.to_string(),
            values: [None, None, None],
            status: SystemStatus::Nominal,
        }
    }

    /// Setzt Wert für einen der drei redundanten Kanäle
    pub fn set_channel(&mut self, channel: usize, value: T) {
        if channel < 3 {
            self.values[channel] = Some(value);
        }
    }

    /// TMR-Voting: Mehrheitsentscheidung
    pub fn vote(&self) -> Option<T> {
        let valid: Vec<&T> = self.values.iter().filter_map(|v| v.as_ref()).collect();

        match valid.len() {
            0 => None,
            1 => Some(valid[0].clone()),
            2 => {
                if valid[0] == valid[1] {
                    Some(valid[0].clone())
                } else {
                    Some(valid[0].clone()) // Nehme ersten bei Uneinigkeit
                }
            }
            3 => {
                // Mehrheitsentscheidung
                if valid[0] == valid[1] || valid[0] == valid[2] {
                    Some(valid[0].clone())
                } else if valid[1] == valid[2] {
                    Some(valid[1].clone())
                } else {
                    Some(valid[0].clone()) // Kein Konsens, nehme ersten
                }
            }
            _ => None,
        }
    }

    /// Prüft Konsistenz und aktualisiert Status
    pub fn check_health(&mut self) {
        let valid_count = self.values.iter().filter(|v| v.is_some()).count();
        let all_equal = {
            let valid: Vec<&T> = self.values.iter().filter_map(|v| v.as_ref()).collect();
            valid.windows(2).all(|w| w[0] == w[1])
        };

        self.status = match (valid_count, all_equal) {
            (3, true) => SystemStatus::Nominal,
            (3, false) => SystemStatus::Warning, // Disagreement
            (2, _) => SystemStatus::Warning,      // Ein Kanal ausgefallen
            (1, _) => SystemStatus::Fault,        // Nur noch ein Kanal
            (0, _) => SystemStatus::Critical,     // Totalausfall
        };
    }
}

/// Watchdog-Timer für Systemüberwachung
#[derive(Debug)]
pub struct Watchdog {
    pub name: String,
    pub timeout: Duration,
    pub last_kick: Instant,
    pub triggered: bool,
}

impl Watchdog {
    pub fn new(name: &str, timeout_ms: u64) -> Self {
        Self {
            name: name.to_string(),
            timeout: Duration::from_millis(timeout_ms),
            last_kick: Instant::now(),
            triggered: false,
        }
    }

    /// "Kick" den Watchdog um Reset zu signalisieren
    pub fn kick(&mut self) {
        self.last_kick = Instant::now();
        self.triggered = false;
    }

    /// Prüft ob Timeout abgelaufen
    pub fn check(&mut self) -> bool {
        if self.last_kick.elapsed() > self.timeout {
            self.triggered = true;
        }
        self.triggered
    }
}

/// FDIR-Manager für das gesamte System
#[derive(Debug)]
pub struct FDIRManager {
    pub watchdog: Watchdog,
    pub system_status: SystemStatus,
    pub fault_count: u32,
    pub recovery_attempts: u32,
    pub max_recovery_attempts: u32,
}

impl FDIRManager {
    pub fn new() -> Self {
        Self {
            watchdog: Watchdog::new("MainLoop", 5000),
            system_status: SystemStatus::Nominal,
            fault_count: 0,
            recovery_attempts: 0,
            max_recovery_attempts: 3,
        }
    }

    /// Führt FDIR-Zyklus aus
    pub fn run_cycle(&mut self) {
        // Watchdog prüfen
        if self.watchdog.check() {
            self.handle_fault("Watchdog timeout");
        }
    }

    /// Behandelt erkannten Fehler
    pub fn handle_fault(&mut self, reason: &str) {
        self.fault_count += 1;
        println!("⚠️ FDIR: Fault detected - {}", reason);

        if self.recovery_attempts < self.max_recovery_attempts {
            self.attempt_recovery();
        } else {
            self.system_status = SystemStatus::Critical;
            println!("🔴 FDIR: System CRITICAL - Max recovery attempts exceeded");
        }
    }

    /// Versucht System-Recovery
    fn attempt_recovery(&mut self) {
        self.recovery_attempts += 1;
        println!(
            "🔧 FDIR: Recovery attempt {}/{}",
            self.recovery_attempts, self.max_recovery_attempts
        );

        // Reset Watchdog
        self.watchdog.kick();
        self.system_status = SystemStatus::Warning;
    }

    /// Meldet erfolgreiche Operation (kickt Watchdog)
    pub fn report_nominal(&mut self) {
        self.watchdog.kick();
        if self.system_status == SystemStatus::Warning {
            self.system_status = SystemStatus::Nominal;
            println!("✅ FDIR: System recovered to nominal");
        }
    }

    /// Prüft ob System noch operabel ist
    pub fn is_operational(&self) -> bool {
        self.system_status != SystemStatus::Critical
    }
}

impl Default for FDIRManager {
    fn default() -> Self {
        Self::new()
    }
}

/// MTBF-Berechnung (für Statistik)
/// MTBF = 1 / λ_system
pub fn calculate_mtbf(failure_rate: f64) -> f64 {
    if failure_rate > 0.0 {
        1.0 / failure_rate
    } else {
        f64::INFINITY
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_tmr_voting() {
        let mut subsys: RedundantSubsystem<i32> = RedundantSubsystem::new("Test");
        subsys.set_channel(0, 42);
        subsys.set_channel(1, 42);
        subsys.set_channel(2, 99); // Fehlerhafter Wert

        let result = subsys.vote();
        assert_eq!(result, Some(42)); // Mehrheit gewinnt
    }

    #[test]
    fn test_health_check() {
        let mut subsys: RedundantSubsystem<i32> = RedundantSubsystem::new("Test");
        subsys.set_channel(0, 1);
        subsys.set_channel(1, 1);
        subsys.set_channel(2, 1);
        subsys.check_health();
        assert_eq!(subsys.status, SystemStatus::Nominal);

        subsys.values[2] = None; // Ein Kanal fällt aus
        subsys.check_health();
        assert_eq!(subsys.status, SystemStatus::Warning);
    }

    #[test]
    fn test_mtbf() {
        let mtbf = calculate_mtbf(0.001); // 0.1% Ausfallrate pro Stunde
        assert_eq!(mtbf, 1000.0); // 1000 Stunden MTBF
    }
}
