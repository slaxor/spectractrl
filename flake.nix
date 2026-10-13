{
  description = "spectractrl";
  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-26.05";
    rust-overlay.url = "github:oxalica/rust-overlay";
  };
  outputs =
    { nixpkgs, rust-overlay, ... }:
    let
      pkgs = import nixpkgs {
        system = "x86_64-linux";
        overlays = [ rust-overlay.overlays.default ];
      };
      rust = pkgs.rust-bin.stable.latest.default.override {
        targets = [ "thumbv6m-none-eabi" ];
        extensions = [
          "rust-src"
          "rust-analyzer"
        ];
      };
    in
    {
      devShells.x86_64-linux.default = pkgs.mkShell {
        packages = with pkgs; [
          rust
          llvm
          just
          probe-rs-tools
          flip-link
        ];
      };
    };
}
