//! `memory.x` を OUT_DIR にコピーしてリンカの検索パスに載せる。
//! 単一クレートならリンカの cwd がクレートルートになるので不要だが、ワークスペースでは
//! cwd がワークスペースルートになり `link.x` の `INCLUDE memory.x` が解決できなくなる。
//! 詳細: https://github.com/rust-lang/cargo/issues/9537

use std::{env, fs, path::PathBuf};

fn main() {
    let out = PathBuf::from(env::var_os("OUT_DIR").unwrap());
    fs::copy("memory.x", out.join("memory.x")).unwrap();
    println!("cargo:rustc-link-search={}", out.display());
    println!("cargo:rerun-if-changed=memory.x");
    println!("cargo:rerun-if-changed=build.rs");
}
