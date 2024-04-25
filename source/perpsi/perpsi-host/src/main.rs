use perpsi_host_client::PerpsiClient;

#[tokio::main]
async fn main() {
    println!("Hello, world!");
    let client = PerpsiClient::new();
    println!("Connected! Pinging 42");
    let ping = client.ping(42).await.unwrap();
    println!("Got: {ping}.");
}
