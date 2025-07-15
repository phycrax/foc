#![no_std]
#![warn(missing_docs)]
#![cfg_attr(not(doctest), doc = include_str!("../README.md"))]
#![cfg_attr(docsrs, feature(doc_cfg))]

//! ## Feature flags
#![doc = document_features::document_features!(feature_label = r#"<span class="stab portability"><code>{feature}</code></span>"#)]

pub mod park_clarke;
pub mod pid;
pub mod pwm;

const FRAC_1_SQRT_3: f32 = 0.57735026_f32;
const SQRT_3: f32 = 1.7320508_f32;
