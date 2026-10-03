//! minishirasu-firm のうち、レジスタに触らない部分。ホストでテストする
#![cfg_attr(not(test), no_std)]

pub mod pwm;
pub mod sense;
