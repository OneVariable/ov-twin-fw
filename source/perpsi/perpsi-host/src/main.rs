use std::io::{stdout, Write};

use perpsi_host_client::{PerpsiClient, icd};

#[tokio::main]
async fn main() {
    println!("Connecting...");
    let client = PerpsiClient::new();
    println!("Connected! Pinging 42");
    let ping = client.ping(42).await.unwrap();
    println!("Got: {ping}.");
    let uid = client.get_id().await.unwrap();
    println!("ID: {uid:016X}");
    println!();

    // Begin repl...
    loop {
        print!("> ");
        stdout().flush().unwrap();
        let line = read_line().await;
        let parts: Vec<&str> = line.split_whitespace().collect();
        match parts.as_slice() {
            ["ping"] => {
                let ping = client.ping(42).await.unwrap();
                println!("Got: {ping}.");
            }
            ["ping", n] => {
                let Ok(idx) = n.parse::<u32>() else {
                    println!("Bad u32: '{n}'");
                    continue;
                };
                let ping = client.ping(idx).await.unwrap();
                println!("Got: {ping}.");
            }
            ["rgb", pos, r, g, b] => {
                let (Ok(pos), Ok(r), Ok(g), Ok(b)) = (pos.parse(), r.parse(), g.parse(), b.parse()) else {
                    panic!();
                };
                client.set_rgb_single(pos, r, g, b).await.unwrap();
            }
            ["rgball", r, g, b] => {
                let (Ok(r), Ok(g), Ok(b)) = (r.parse(), g.parse(), b.parse()) else {
                    panic!();
                };
                client.set_all_rgb_single(r, g, b).await.unwrap();
            }
            ["accel", "start", ms, range] => {
                let Ok(ms) = ms.parse::<u32>() else {
                    println!("Bad ms: {ms}");
                    continue;
                };
                let range = match *range {
                    "2" => icd::AccelRange::G2,
                    "4" => icd::AccelRange::G4,
                    "8" => icd::AccelRange::G8,
                    "16" => icd::AccelRange::G16,
                    _ => {
                        println!("Bad range: {range}");
                        continue;
                    }
                };

                client.start_accelerometer(ms, range).await.unwrap();
                println!("Started!");
            }
            ["accel", "stop"] => {
                let res = client.stop_accelerometer().await.unwrap();
                println!("Stopped: {res}");
            }
            other => {
                println!("Error, didn't understand '{other:?};");
            }
        }
    }
}

async fn read_line() -> String {
    tokio::task::spawn_blocking(|| {
        let mut line = String::new();
        std::io::stdin().read_line(&mut line).unwrap();
        line
    })
    .await
    .unwrap()
}
