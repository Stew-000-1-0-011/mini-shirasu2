//! minishirasu-firm のうち、レジスタに触らない部分。ホストでテストする
#![cfg_attr(not(test), no_std)]

pub mod config;
pub mod protocol;
pub mod pwm;
pub mod sense;
pub mod txbuf;
