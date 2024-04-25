use perpsi_icd::{PingEndpoint, SetAllLedEndpoint, GetUniqueIdEndpoint, SetSingleLedEndpoint, SingleLed, Rgb8};
use postcard_rpc::host_client::{HostClient, HostErr};
use postcard_rpc::standard_icd::{WireError, ERROR_PATH};

pub struct PerpsiClient {
    client: HostClient<WireError>,
}

pub type Result<T> = std::result::Result<T, HostErr<WireError>>;

impl PerpsiClient {
    pub fn new() -> Self {
        let client =
            HostClient::new_raw_nusb(|d| d.product_string() == Some("ov-twin"), ERROR_PATH, 8);
        Self { client }
    }

    pub async fn ping(&self, id: u32) -> Result<u32> {
        self.client.send_resp::<PingEndpoint>(&id).await
    }

    pub async fn get_id(&self) -> Result<u64> {
        self.client.send_resp::<GetUniqueIdEndpoint>(&()).await
    }

    pub async fn set_rgb_single(&self, position: u32, r: u8, g: u8, b: u8) -> Result<()> {
        self.client.send_resp::<SetSingleLedEndpoint>(&SingleLed {
            position, rgb: Rgb8 { r, g, b },
        }).await?.unwrap();
        Ok(())
    }

    pub async fn set_all_rgb_single(&self, r: u8, g: u8, b: u8) -> Result<()> {
        self.client.send_resp::<SetAllLedEndpoint>(&[Rgb8 { r, g, b }; 24]).await
    }
}
