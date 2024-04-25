#![no_std]

use postcard::experimental::schema::Schema;
use postcard_rpc::endpoint;
use serde::{Deserialize, Serialize};

endpoint!(PingEndpoint, u32, u32, "ping");
endpoint!(GetUniqueIdEndpoint, (), u64, "unique_id/get");

endpoint!(SetSingleLedEndpoint, SingleLed, Result<(), BadPositionError>, "led/set_one");
endpoint!(SetAllLedEndpoint, [Rgb8; 24], (), "led/set_all");

#[derive(Serialize, Deserialize, Schema, Debug, PartialEq)]
pub struct SingleLed {
    pub position: u32,
    pub rgb: Rgb8,
}

#[derive(Serialize, Deserialize, Schema, Debug, PartialEq, Copy, Clone)]
pub struct Rgb8 {
    pub r: u8,
    pub g: u8,
    pub b: u8
}

#[derive(Serialize, Deserialize, Schema, Debug, PartialEq)]
pub struct BadPositionError;
