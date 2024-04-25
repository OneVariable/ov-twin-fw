#![no_std]

use postcard_rpc::endpoint;

endpoint!(PingEndpoint, u32, u32, "ping");
