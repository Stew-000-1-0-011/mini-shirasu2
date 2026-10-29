//! `memory.x` を OUT_DIR にコピーしてリンカの検索パスに載せる。
//! 単一クレートならリンカの cwd がクレートルートになるので不要だが、ワークスペースでは
//! cwd がワークスペースルートになり `link.x` の `INCLUDE memory.x` が解決できなくなる。
//! 詳細: https://github.com/rust-lang/cargo/issues/9537
//!
//! あわせて、書き込んだファームの新しさを起動ログで確かめられるよう、
//! gitのブランチ・コミット・未コミットの変更の有無を FIRM_GIT_* として埋め込む

use std::{env, fs, path::PathBuf, process::Command};

/// gitコマンドの出力の1行目。gitがない、リポジトリでないなどで失敗したらNone
fn git(args: &[&str]) -> Option<String> {
    let out = Command::new("git").args(args).output().ok()?;
    if !out.status.success() {
        return None;
    }
    Some(String::from_utf8_lossy(&out.stdout).lines().next().unwrap_or("").trim().to_string())
}

fn main() {
    let out = PathBuf::from(env::var_os("OUT_DIR").unwrap());
    fs::copy("memory.x", out.join("memory.x")).unwrap();
    println!("cargo:rustc-link-search={}", out.display());
    println!("cargo:rerun-if-changed=memory.x");
    println!("cargo:rerun-if-changed=build.rs");

    let commit = git(&["rev-parse", "--short=7", "HEAD"]).unwrap_or_else(|| "unknown".into());
    let branch = git(&["rev-parse", "--abbrev-ref", "HEAD"]).unwrap_or_else(|| "unknown".into());
    // 出力が空ならきれい。gitが失敗したら分からないので "?" にする
    let dirty = match git(&["status", "--porcelain", "--untracked-files=no"]) {
        Some(s) if s.is_empty() => "",
        Some(_) => "-dirty",
        None => "?",
    };
    println!("cargo:rustc-env=FIRM_GIT_COMMIT={commit}");
    println!("cargo:rustc-env=FIRM_GIT_BRANCH={branch}");
    println!("cargo:rustc-env=FIRM_GIT_DIRTY={dirty}");

    // コミットやブランチが変わったとき、ソースを書き換えたときに埋め込み直す
    if let Some(git_dir) = git(&["rev-parse", "--absolute-git-dir"]) {
        println!("cargo:rerun-if-changed={git_dir}/HEAD");
        println!("cargo:rerun-if-changed={git_dir}/index");
        if let Some(head_ref) = git(&["symbolic-ref", "-q", "HEAD"]) {
            println!("cargo:rerun-if-changed={git_dir}/{head_ref}");
        }
    }
    println!("cargo:rerun-if-changed=src");
}
