use perpsi_icd::PingEndpoint;
use postcard_rpc::host_client::{HostClient, HostErr};
use postcard_rpc::standard_icd::{WireError, ERROR_PATH};

pub struct PerpsiClient {
    client: HostClient<WireError>,
}

pub type Result<T> = std::result::Result<T, HostErr<WireError>>;

impl PerpsiClient {
    pub fn new() -> Self {
        let client = HostClient::new_raw_nusb(
            |d| d.product_string() == Some("ov-twin"),
            ERROR_PATH,
            8,
        );
        Self { client }
    }

    pub async fn ping(&self, id: u32) -> Result<u32> {
        self.client.send_resp::<PingEndpoint>(&id).await
    }
}
