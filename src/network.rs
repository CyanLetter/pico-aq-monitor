//! Network operations for API submission.

use crate::types::{NetworkStatus, SensorPayload};
use defmt::Debug2Format;
use embassy_net::dns::DnsSocket;
use embassy_net::tcp::client::{TcpClient, TcpClientState};
use embassy_rp::clocks::RoscRng;
use embassy_time::Duration;
use reqwless::client::{HttpClient, TlsConfig, TlsVerify};
use reqwless::request::{Method, RequestBuilder};

// API configuration loaded from .env file at build time
const API_ENDPOINT: &str = env!("API_ENDPOINT");
const API_KEY: &str = env!("API_KEY");

/// Network timeout in seconds
const HTTP_TIMEOUT_SECS: u64 = 10;

/// Check if network is currently connected
pub fn check_network_status(stack: embassy_net::Stack<'static>) -> NetworkStatus {
    if stack.is_link_up() && stack.config_v4().is_some() {
        NetworkStatus::Connected
    } else {
        NetworkStatus::Disconnected
    }
}

/// Internal implementation of HTTP request with TLS support
async fn do_http_request(
    stack: embassy_net::Stack<'static>,
    json_bytes: &[u8],
    rng: &mut RoscRng,
) -> Result<u16, &'static str> {
    let mut rx_buffer = [0; 4096];
    let mut tls_read_buffer = [0; 16384];
    let mut tls_write_buffer = [0; 4096];

    let client_state = TcpClientState::<1, 4096, 4096>::new();
    let tcp_client = TcpClient::new(stack, &client_state);
    let dns_client = DnsSocket::new(stack);

    // Configure TLS (skip certificate verification for embedded device)
    let seed = rng.next_u64();
    let tls_config = TlsConfig::new(
        seed,
        &mut tls_read_buffer,
        &mut tls_write_buffer,
        TlsVerify::None,
    );

    let mut http_client = HttpClient::new_with_tls(&tcp_client, &dns_client, tls_config);

    defmt::info!("Creating request for endpoint: {}", API_ENDPOINT);
    log::info!("Creating request for endpoint: {}", API_ENDPOINT);

    let request = match http_client.request(Method::POST, API_ENDPOINT).await {
        Ok(req) => req,
        Err(e) => {
            defmt::error!("Failed to create HTTP request: {}", Debug2Format(&e));
            log::error!("Failed to create HTTP request: {:?}", e);
            return Err("Failed to create HTTP request");
        }
    };

    defmt::info!("Request created, sending...");
    log::info!("Request created, sending...");

    let headers = [("Content-Type", "application/json"), ("X-API-Key", API_KEY)];
    let mut request = request.headers(&headers).body(json_bytes);

    let response = match request.send(&mut rx_buffer).await {
        Ok(resp) => resp,
        Err(e) => {
            defmt::error!("Failed to send HTTP request: {}", Debug2Format(&e));
            log::error!("Failed to send HTTP request: {:?}", e);
            return Err("Failed to send HTTP request");
        }
    };

    Ok(response.status.0)
}

/// Submit sensor data to API endpoint with timeout
pub async fn submit_sensor_data(
    stack: embassy_net::Stack<'static>,
    payload: &SensorPayload,
    rng: &mut RoscRng,
) -> Result<(), &'static str> {
    // Check network status first
    if check_network_status(stack) == NetworkStatus::Disconnected {
        return Err("Network disconnected");
    }

    let json = payload.to_json();
    let json_bytes = json.as_bytes();

    defmt::debug!("Submitting to API: {}", json.as_str());

    // Apply timeout to the entire HTTP operation
    let timeout_duration = Duration::from_secs(HTTP_TIMEOUT_SECS);
    let result = embassy_time::with_timeout(
        timeout_duration,
        do_http_request(stack, json_bytes, rng),
    )
    .await;

    match result {
        Ok(Ok(status)) if status >= 200 && status < 300 => {
            defmt::info!("API submission successful (status {})", status);
            log::info!("API submission successful (status {})", status);
            Ok(())
        }
        Ok(Ok(status)) => {
            defmt::warn!("API submission failed (status {})", status);
            log::warn!("API submission failed (status {})", status);
            Err("API returned error status")
        }
        Ok(Err(e)) => {
            defmt::warn!("HTTP request failed: {}", e);
            log::warn!("HTTP request failed: {}", e);
            Err(e)
        }
        Err(_) => {
            defmt::warn!("HTTP request timed out after {} seconds", HTTP_TIMEOUT_SECS);
            log::warn!("HTTP request timed out after {} seconds", HTTP_TIMEOUT_SECS);
            Err("Request timed out")
        }
    }
}
