use pyo3::prelude::*;

mod types;
mod path_validator;
mod safe_map_filter;
mod connector_planner;
mod zigzag;
mod spiral;

#[pyfunction]
fn py_health_check() -> &'static str {
    "ok"
}

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
