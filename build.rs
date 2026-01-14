//! This build script copies the `memory.x` file from the crate root into
//! a directory where the linker can always find it at build time.
//! For many projects this is optional, as the linker always searches the
//! project root directory -- wherever `Cargo.toml` is. However, if you
//! are using a workspace or have a more complicated build setup, this
//! build script becomes required. Additionally, by requesting that
//! Cargo re-run the build script whenever `memory.x` is changed,
//! updating `memory.x` ensures a rebuild of the application with the
//! new memory settings.
//!
//! This script also loads WiFi and API configuration from a `.env` file.

use std::env;
use std::fs::File;
use std::io::Write;
use std::path::PathBuf;

fn main() {
    // Load environment variables from .env file
    println!("cargo:rerun-if-changed=.env");

    if let Ok(env_path) = dotenvy::dotenv() {
        println!("cargo:warning=Loaded config from {:?}", env_path);
    } else {
        println!("cargo:warning=No .env file found, using environment variables");
    }

    // Pass WiFi configuration to the compiler
    if let Ok(ssid) = env::var("WIFI_SSID") {
        println!("cargo:rustc-env=WIFI_SSID={}", ssid);
    } else {
        panic!("WIFI_SSID not set in .env file or environment");
    }

    if let Ok(password) = env::var("WIFI_PASSWORD") {
        println!("cargo:rustc-env=WIFI_PASSWORD={}", password);
    } else {
        panic!("WIFI_PASSWORD not set in .env file or environment");
    }

    // Pass API configuration to the compiler
    if let Ok(endpoint) = env::var("API_ENDPOINT") {
        println!("cargo:rustc-env=API_ENDPOINT={}", endpoint);
    } else {
        panic!("API_ENDPOINT not set in .env file or environment");
    }

    if let Ok(key) = env::var("API_KEY") {
        println!("cargo:rustc-env=API_KEY={}", key);
    } else {
        panic!("API_KEY not set in .env file or environment");
    }

    // Put `memory.x` in our output directory and ensure it's
    // on the linker search path.
    let out = &PathBuf::from(env::var_os("OUT_DIR").unwrap());
    File::create(out.join("memory.x"))
        .unwrap()
        .write_all(include_bytes!("memory.x"))
        .unwrap();
    println!("cargo:rustc-link-search={}", out.display());

    // By default, Cargo will re-run a build script whenever
    // any file in the project changes. By specifying `memory.x`
    // here, we ensure the build script is only re-run when
    // `memory.x` is changed.
    println!("cargo:rerun-if-changed=memory.x");

    println!("cargo:rustc-link-arg-bins=--nmagic");
    println!("cargo:rustc-link-arg-bins=-Tlink.x");
    println!("cargo:rustc-link-arg-bins=-Tlink-rp.x");
    println!("cargo:rustc-link-arg-bins=-Tdefmt.x");
}
