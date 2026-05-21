cmake_minimum_required(VERSION 3.8)
project(mower_coverage_core)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter)

ament_get_python_install_dir(PYTHON_INSTALL_DIR)
_ament_cmake_python_register_environment_hook()

# Locate cargo so we can pass it explicitly to maturin's subprocess environment.
# cmake's execute_process does not always inherit the login-shell PATH, so cargo
# (installed at /usr/local/cargo/bin by the devcontainer Dockerfile) may be
# missing from the subprocess PATH even when the interactive shell can find it.
find_program(CARGO_EXECUTABLE cargo
  HINTS /usr/local/cargo/bin $ENV{CARGO_HOME}/bin $ENV{HOME}/.cargo/bin
)
if(NOT CARGO_EXECUTABLE)
  message(FATAL_ERROR "cargo not found. "
    "Install the Rust toolchain: "
    "curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh")
endif()
get_filename_component(_cargo_bin_dir "${CARGO_EXECUTABLE}" DIRECTORY)

# Build the maturin wheel at CMake configure time.
# Explicitly extend PATH with the cargo bin directory so that maturin's
# internal `cargo metadata` / `cargo build` invocations can find the toolchain
# regardless of how cmake was launched (interactive shell vs. colcon subprocess).
execute_process(
  COMMAND ${CMAKE_COMMAND} -E env
          "PATH=${_cargo_bin_dir}:$ENV{PATH}"
          "CARGO_HOME=$ENV{CARGO_HOME}"
          "RUSTUP_HOME=$ENV{RUSTUP_HOME}"
          ${Python3_EXECUTABLE} -m maturin build --release
          --out "${CMAKE_CURRENT_BINARY_DIR}/wheels"
  WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
  RESULT_VARIABLE _maturin_result
  COMMAND_ECHO STDOUT
)
if(NOT _maturin_result EQUAL 0)
  message(FATAL_ERROR "maturin build failed (exit code ${_maturin_result}). "
    "Ensure Rust toolchain and maturin are installed: "
    "curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh && pip install maturin")
endif()

file(GLOB _maturin_wheels "${CMAKE_CURRENT_BINARY_DIR}/wheels/*.whl")
list(LENGTH _maturin_wheels _maturin_wheel_count)
if(NOT _maturin_wheel_count EQUAL 1)
  message(FATAL_ERROR "Expected exactly one mower_coverage_core wheel in "
    "${CMAKE_CURRENT_BINARY_DIR}/wheels, found ${_maturin_wheel_count}")
endif()
list(GET _maturin_wheels 0 _maturin_wheel)

# Install the wheel via pip at install time
install(CODE
  "execute_process(
    COMMAND ${Python3_EXECUTABLE} -m pip install --no-deps
            --upgrade --force-reinstall
            --target \"${CMAKE_INSTALL_PREFIX}/${PYTHON_INSTALL_DIR}\"
            \"${_maturin_wheel}\"
    RESULT_VARIABLE _pip_result
  )
  if(NOT _pip_result EQUAL 0)
    message(FATAL_ERROR \"pip install of mower_coverage_core wheel failed\")
  endif()"
)

ament_package()
