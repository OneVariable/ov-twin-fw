use std::time::Duration;

use workbook_host_client::client::WorkbookClient;
use tokio::time::interval;

#[tokio::main]
pub async fn main() {
    let client = WorkbookClient::new();
    let mut ticker = interval(Duration::from_millis(250));

    for i in 0..10 {
        ticker.tick().await;
        print!("Pinging with {i}... ");
        let res = client.ping(i).await.unwrap();
        println!("got {res}!");
        assert_eq!(res, i);
    }
}
