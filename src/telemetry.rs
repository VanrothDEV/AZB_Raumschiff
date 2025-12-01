//! Telemetrie-Modul: TT&C (Telemetry, Telecommand & Data Handling)
//!
//! Implementiert:
//! - Telemetrie-Pakete mit CRC
//! - Event-Logging
//! - Daten-Serialisierung

use std::time::{SystemTime, UNIX_EPOCH};

/// CRC-64 Polynom (vereinfacht)
const CRC_POLY: u64 = 0x42F0E1EBA9EA3693;

/// Telemetrie-Paket
#[derive(Debug, Clone)]
pub struct TelemetryPacket {
    /// Zeitstempel (Unix-Zeit in ms)
    pub timestamp: u64,
    /// Paket-ID
    pub packet_id: u32,
    /// Subsystem-ID
    pub subsystem: SubsystemId,
    /// Nutzdaten
    pub payload: TelemetryPayload,
    /// CRC-64 Prüfsumme
    pub crc: u64,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum SubsystemId {
    GNC = 1,
    FDIR = 2,
    Propulsion = 3,
    Thermal = 4,
    Power = 5,
    Communication = 6,
}

#[derive(Debug, Clone)]
pub enum TelemetryPayload {
    /// Position und Geschwindigkeit
    Navigation {
        position: [f64; 3],
        velocity: [f64; 3],
    },
    /// Systemstatus
    Status {
        phase: u8,
        fuel_percent: f32,
        system_health: u8,
    },
    /// Sensorwerte
    Sensors {
        temperature: f32,
        pressure: f32,
        radiation: f32,
    },
    /// Ereignis
    Event {
        event_code: u16,
        message: String,
    },
}

impl TelemetryPacket {
    pub fn new(packet_id: u32, subsystem: SubsystemId, payload: TelemetryPayload) -> Self {
        let timestamp = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .unwrap()
            .as_millis() as u64;

        let mut packet = Self {
            timestamp,
            packet_id,
            subsystem,
            payload,
            crc: 0,
        };
        packet.crc = packet.calculate_crc();
        packet
    }

    /// Berechnet CRC-64 über Paketdaten (vereinfacht)
    fn calculate_crc(&self) -> u64 {
        let mut crc: u64 = 0xFFFFFFFFFFFFFFFF;

        // Timestamp einbeziehen
        for byte in self.timestamp.to_le_bytes() {
            crc = Self::crc_byte(crc, byte);
        }

        // Packet-ID
        for byte in self.packet_id.to_le_bytes() {
            crc = Self::crc_byte(crc, byte);
        }

        // Subsystem
        crc = Self::crc_byte(crc, self.subsystem as u8);

        crc
    }

    fn crc_byte(crc: u64, byte: u8) -> u64 {
        let mut c = crc ^ (byte as u64);
        for _ in 0..8 {
            if c & 1 != 0 {
                c = (c >> 1) ^ CRC_POLY;
            } else {
                c >>= 1;
            }
        }
        c
    }

    /// Validiert CRC
    pub fn validate(&self) -> bool {
        self.crc == self.calculate_crc()
    }

    /// Serialisiert zu Bytes (vereinfachte Binär-Serialisierung)
    pub fn to_bytes(&self) -> Vec<u8> {
        let mut bytes = Vec::new();

        // Header
        bytes.extend_from_slice(&self.timestamp.to_le_bytes());
        bytes.extend_from_slice(&self.packet_id.to_le_bytes());
        bytes.push(self.subsystem as u8);

        // Payload-Typ + Daten
        match &self.payload {
            TelemetryPayload::Navigation { position, velocity } => {
                bytes.push(0x01);
                for v in position {
                    bytes.extend_from_slice(&v.to_le_bytes());
                }
                for v in velocity {
                    bytes.extend_from_slice(&v.to_le_bytes());
                }
            }
            TelemetryPayload::Status {
                phase,
                fuel_percent,
                system_health,
            } => {
                bytes.push(0x02);
                bytes.push(*phase);
                bytes.extend_from_slice(&fuel_percent.to_le_bytes());
                bytes.push(*system_health);
            }
            TelemetryPayload::Sensors {
                temperature,
                pressure,
                radiation,
            } => {
                bytes.push(0x03);
                bytes.extend_from_slice(&temperature.to_le_bytes());
                bytes.extend_from_slice(&pressure.to_le_bytes());
                bytes.extend_from_slice(&radiation.to_le_bytes());
            }
            TelemetryPayload::Event {
                event_code,
                message,
            } => {
                bytes.push(0x04);
                bytes.extend_from_slice(&event_code.to_le_bytes());
                let msg_bytes = message.as_bytes();
                bytes.extend_from_slice(&(msg_bytes.len() as u16).to_le_bytes());
                bytes.extend_from_slice(msg_bytes);
            }
        }

        // CRC am Ende
        bytes.extend_from_slice(&self.crc.to_le_bytes());

        bytes
    }
}

/// Telemetrie-Logger
pub struct TelemetryLogger {
    packets: Vec<TelemetryPacket>,
    next_id: u32,
}

impl TelemetryLogger {
    pub fn new() -> Self {
        Self {
            packets: Vec::new(),
            next_id: 1,
        }
    }

    /// Loggt Navigationsdaten
    pub fn log_navigation(&mut self, position: [f64; 3], velocity: [f64; 3]) {
        let payload = TelemetryPayload::Navigation { position, velocity };
        self.log(SubsystemId::GNC, payload);
    }

    /// Loggt Systemstatus
    pub fn log_status(&mut self, phase: u8, fuel_percent: f32, system_health: u8) {
        let payload = TelemetryPayload::Status {
            phase,
            fuel_percent,
            system_health,
        };
        self.log(SubsystemId::FDIR, payload);
    }

    /// Loggt Ereignis
    pub fn log_event(&mut self, subsystem: SubsystemId, event_code: u16, message: &str) {
        let payload = TelemetryPayload::Event {
            event_code,
            message: message.to_string(),
        };
        self.log(subsystem, payload);
    }

    fn log(&mut self, subsystem: SubsystemId, payload: TelemetryPayload) {
        let packet = TelemetryPacket::new(self.next_id, subsystem, payload);
        self.packets.push(packet);
        self.next_id += 1;
    }

    /// Gibt alle Pakete zurück
    pub fn get_packets(&self) -> &[TelemetryPacket] {
        &self.packets
    }

    /// Exportiert Telemetrie als Text
    pub fn export_summary(&self) -> String {
        let mut output = String::new();
        output.push_str("=== TELEMETRY LOG ===\n");
        output.push_str(&format!("Total packets: {}\n\n", self.packets.len()));

        for packet in &self.packets {
            output.push_str(&format!(
                "[{}] #{} {:?}: ",
                packet.timestamp, packet.packet_id, packet.subsystem
            ));
            match &packet.payload {
                TelemetryPayload::Navigation { position, velocity } => {
                    output.push_str(&format!(
                        "NAV pos=[{:.0}, {:.0}, {:.0}]m vel=[{:.1}, {:.1}, {:.1}]m/s\n",
                        position[0],
                        position[1],
                        position[2],
                        velocity[0],
                        velocity[1],
                        velocity[2]
                    ));
                }
                TelemetryPayload::Status {
                    phase,
                    fuel_percent,
                    system_health,
                } => {
                    output.push_str(&format!(
                        "STATUS phase={} fuel={:.1}% health={}\n",
                        phase, fuel_percent, system_health
                    ));
                }
                TelemetryPayload::Sensors {
                    temperature,
                    pressure,
                    radiation,
                } => {
                    output.push_str(&format!(
                        "SENSORS temp={:.1}°C press={:.1}kPa rad={:.2}mSv\n",
                        temperature, pressure, radiation
                    ));
                }
                TelemetryPayload::Event {
                    event_code,
                    message,
                } => {
                    output.push_str(&format!("EVENT [{}] {}\n", event_code, message));
                }
            }
        }

        output
    }
}

impl Default for TelemetryLogger {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_packet_crc() {
        let packet = TelemetryPacket::new(
            1,
            SubsystemId::GNC,
            TelemetryPayload::Navigation {
                position: [1.0, 2.0, 3.0],
                velocity: [4.0, 5.0, 6.0],
            },
        );
        assert!(packet.validate());
    }

    #[test]
    fn test_serialization() {
        let packet = TelemetryPacket::new(
            1,
            SubsystemId::FDIR,
            TelemetryPayload::Status {
                phase: 2,
                fuel_percent: 75.5,
                system_health: 100,
            },
        );
        let bytes = packet.to_bytes();
        assert!(!bytes.is_empty());
    }

    #[test]
    fn test_logger() {
        let mut logger = TelemetryLogger::new();
        logger.log_navigation([1e6, 2e6, 3e6], [100.0, 200.0, 300.0]);
        logger.log_event(SubsystemId::GNC, 1001, "Engine ignition");
        assert_eq!(logger.get_packets().len(), 2);
    }
}
