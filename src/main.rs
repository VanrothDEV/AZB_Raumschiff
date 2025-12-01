//! AZB_Raumschiff – Autonome Mondlandung
//!
//! Hauptprogramm für die Simulation einer Mondmission.
//!
//! Usage:
//!   cargo run              # Standardmission
//!   cargo run -- --fast    # Schnelle Simulation (größerer Zeitschritt)
//!   cargo run -- --test    # Kurzer Test (10 Minuten simuliert)

use azb_raumschiff::simulation::{MoonMissionSim, SimConfig, run_moon_mission};
use std::env;

fn main() {
    println!("╔══════════════════════════════════════════════════════════════╗");
    println!("║      🚀 AZB_RAUMSCHIFF ZUR MUTTERERDE 🌙                     ║");
    println!("║      Autonomes Mondlandungsprogramm v0.1.0                   ║");
    println!("╚══════════════════════════════════════════════════════════════╝");
    println!();

    let args: Vec<String> = env::args().collect();

    let result = if args.contains(&"--fast".to_string()) {
        println!("⚡ Schnellmodus aktiviert (dt=5s)");
        println!();
        let config = SimConfig {
            dt: 5.0,           // 5s Zeitschritt
            telemetry_interval: 600.0,
            ..Default::default()
        };
        let mut sim = MoonMissionSim::new(config);
        sim.run()
    } else if args.contains(&"--test".to_string()) {
        println!("🧪 Testmodus (1 Stunde simuliert)");
        println!();
        let config = SimConfig {
            dt: 1.0,
            max_time: 3600.0,    // 1 Stunde
            telemetry_interval: 60.0,
            ..Default::default()
        };
        let mut sim = MoonMissionSim::new(config);
        sim.run()
    } else {
        println!("🎯 Standardmission (kann einige Minuten dauern)");
        println!("   Tipp: `cargo run -- --fast` für schnellere Simulation");
        println!();
        run_moon_mission()
    };

    // Ergebnis ausgeben
    println!();
    println!("════════════════════════════════════════════════════════════════");
    println!("                     MISSION REPORT");
    println!("════════════════════════════════════════════════════════════════");

    if result.success {
        println!("Status:       ✅ ERFOLG - Mondlandung abgeschlossen!");
    } else {
        println!("Status:       ❌ FEHLGESCHLAGEN");
    }

    let hours = result.mission_time / 3600.0;
    let days = hours / 24.0;
    println!("Missionszeit: {:.1} Stunden ({:.2} Tage)", hours, days);

    println!("Treibstoff:   {:.0} kg verbraucht", result.fuel_used);

    let final_pos = result.final_state.position;
    let final_vel = result.final_state.velocity;
    println!(
        "Endposition:  [{:.0}, {:.0}, {:.0}] km",
        final_pos.x / 1000.0,
        final_pos.y / 1000.0,
        final_pos.z / 1000.0
    );
    println!(
        "Endgeschw.:   [{:.1}, {:.1}, {:.1}] m/s (|v| = {:.1} m/s)",
        final_vel.x,
        final_vel.y,
        final_vel.z,
        final_vel.norm()
    );

    println!();
    println!("Telemetrie:   {} Pakete aufgezeichnet", result.telemetry.get_packets().len());
    println!("════════════════════════════════════════════════════════════════");

    // Exit-Code
    std::process::exit(if result.success { 0 } else { 1 });
}
