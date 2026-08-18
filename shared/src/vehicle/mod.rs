pub mod electrics;
pub mod gearbox;
pub mod transform;
pub mod vehicle_meta;

pub use electrics::*;
pub use gearbox::*;
pub use transform::*;
pub use vehicle_meta::*;

use serde::{Deserialize, Serialize};

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct VehicleReset {
    pub vehicle_id: u32,
    pub position: [f32; 3],
    pub rotation: [f32; 4],
}

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct VehicleData {
    pub parts_config: String,
    pub in_game_id: u32,
    pub color: [f32; 8],
    pub palete_0: [f32; 8],
    pub palete_1: [f32; 8],
    pub plate: Option<String>,
    pub name: String,
    pub server_id: u32,
    pub owner: Option<u32>,
    pub position: [f32; 3],
    pub rotation: [f32; 4],
}

/// A single packet that contains all state for one vehicle update.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct VehicleUpdate {
    /// Pose and twist of the sender's centre of gravity.
    pub transform: Transform,
    /// Control inputs, replayed on the receiver.
    pub electrics: Electrics,
    pub gearbox: Gearbox,
    /// Unique vehicle ID on the server.
    pub vehicle_id: u32,
    /// Monotonically increasing counter, used to drop out-of-order packets when
    /// `send_timer` is unavailable.
    pub generation: u64,
    /// Sender wall-clock timestamp in seconds. Subject to cross-machine clock
    /// skew, which is why prediction prefers `send_timer`.
    pub sent_at: f64,
    /// Sender-side monotonic vehicle timer, in seconds, sampled at the physics
    /// step the transform was taken from. The receiver dead-reckons forward
    /// from this, so it must come from the same clock domain as the sample.
    /// Optional for backward compatibility with older Lua clients.
    pub send_timer: Option<f64>,
    /// Sender-side latency estimate in milliseconds: its smoothed RTT to the
    /// server plus the age of its own transform sample. The receiver halves
    /// this to estimate one-way sender-to-server delay.
    /// Optional for backward compatibility with older Lua clients.
    pub ping_ms: Option<f64>,
}

#[derive(Debug, Clone, PartialEq, Deserialize, Serialize)]
pub struct CouplerAttached {
    obj_a: u32,
    obj_b: u32,
    node_a_id: u32,
    node_b_id: u32,
}

#[derive(Debug, Clone, PartialEq, Deserialize, Serialize)]
pub struct CouplerDetached {
    obj_a: u32,
    obj_b: u32,
    node_a_id: u32,
    node_b_id: u32,
}

pub struct ServerSetupResult {
    pub addr: String,
    pub port: u16,
    pub is_upnp: bool,
}
