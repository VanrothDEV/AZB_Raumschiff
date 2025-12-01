//! AZB_Raumschiff – Modulares Raumfahrtsystem
//!
//! Subsysteme nach README.md:
//! - physics: Translationsdynamik, Gravitation, Massenstrom
//! - gnc: Guidance, Navigation & Control (Kalman-Filter, Quaternionen)
//! - fdir: Fault Detection, Isolation & Recovery
//! - telemetry: Telemetrie & Datenhandling
//! - simulation: 6-DOF Simulations-Loop

pub mod physics;
pub mod gnc;
pub mod fdir;
pub mod telemetry;
pub mod simulation;
