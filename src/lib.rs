#![no_std]
#![warn(missing_docs)]
#![cfg_attr(not(doctest), doc = include_str!("../README.md"))]
#![cfg_attr(docsrs, feature(doc_cfg))]

//! ## Feature flags
#![doc = document_features::document_features!(feature_label = r#"<span class="stab portability"><code>{feature}</code></span>"#)]

pub mod park_clarke;
pub mod pid;
pub mod pwm;

#[allow(clippy::excessive_precision)]
const FRAC_1_SQRT_3: f32 = 0.577350269189625764509148780501957456_f32;

#[allow(clippy::excessive_precision)]
const SQRT_3: f32 = 1.732050807568877293527446341505872367_f32;
