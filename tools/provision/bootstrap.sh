#!/usr/bin/env bash
set -euo pipefail

# Ensure required apt packages
sudo apt-get update
sudo apt-get install -y git build-essential curl

# Install Rust toolchain if not present
if ! command -v cargo &>/dev/null; then
	curl https://sh.rustup.rs -sSf | sh -s -- -y
	export PATH="$HOME/.cargo/bin:$PATH"
fi

# Compile psh in release mode
cargo build --release

# Install the compiled binary (will attempt system path then fallback to ~/.local/bin)
cargo run -- install

echo "psh installed successfully."
