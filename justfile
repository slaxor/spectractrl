build:
    cargo build --release

run:
    cargo run --release

doc:
    cargo doc --workspace --no-deps --open

fmt:
    cargo fmt --all

lint:
    cargo clippy --workspace --all-targets -- -D warnings

fix:
    cargo clippy --workspace --all-targets --fix --allow-dirty
    cargo fmt --all
