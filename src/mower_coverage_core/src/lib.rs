//! Coverage path planning core: safe-map filtering, zigzag / spiral
//! generation, path validation and A* connectors. Used from Python through
//! the PyO3 module (feature `python`, the default) and directly from the
//! mower_rs coverage node.

#[cfg(feature = "python")]
use pyo3::prelude::*;

pub mod types;
pub mod path_validator;
pub mod safe_map_filter;
pub mod connector_planner;
pub mod zigzag;
pub mod spiral;

#[cfg(feature = "python")]
#[pyfunction]
fn py_health_check() -> &'static str {
    "ok"
}

#[cfg(feature = "python")]
#[pymodule]
fn mower_coverage_core(m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add_function(wrap_pyfunction!(py_health_check, m)?)?;
    path_validator::register(m)?;
    safe_map_filter::register(m)?;
    connector_planner::register(m)?;
    zigzag::register(m)?;
    spiral::register(m)?;
    Ok(())
}
