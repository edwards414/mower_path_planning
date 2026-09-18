cmake_minimum_required(VERSION 3.8)
project(mower_rs NONE)

find_package(ament_cmake REQUIRED)

# Same toolchain lookup as mower_coverage_core: colcon subprocesses do not
# always inherit the login-shell PATH.
find_program(CARGO_EXECUTABLE cargo
  HINTS /usr/local/cargo/bin $ENV{CARGO_HOME}/bin $ENV{HOME}/.cargo/bin
)
if(NOT CARGO_EXECUTABLE)
  message(FATAL_ERROR "cargo not found. Install the Rust toolchain: "
    "curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh")
endif()
get_filename_component(_cargo_bin_dir "${CARGO_EXECUTABLE}" DIRECTORY)

# r2r's build script reads AMENT_PREFIX_PATH (set up by colcon from the
# <depend>s in package.xml) to find the message typesupport libraries. The
# cargo target dir lives in the colcon build tree so incremental builds and
# the CI cache mount (CARGO_TARGET_DIR is honoured when set) both work.
if(DEFINED ENV{CARGO_TARGET_DIR})
  set(_cargo_target_dir "$ENV{CARGO_TARGET_DIR}/mower_rs")
else()
  set(_cargo_target_dir "${CMAKE_CURRENT_BINARY_DIR}/cargo-target")
endif()

set(MOWER_RS_BINARIES robot_status velocity_command_guard)

add_custom_target(mower_rs_cargo ALL
  COMMAND ${CMAKE_COMMAND} -E env
          "PATH=${_cargo_bin_dir}:$ENV{PATH}"
          "CARGO_TARGET_DIR=${_cargo_target_dir}"
          --unset=MAKEFLAGS --unset=MFLAGS
          ${CARGO_EXECUTABLE} build --release --locked
          --manifest-path "${CMAKE_CURRENT_SOURCE_DIR}/Cargo.toml"
  WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
  COMMENT "cargo build --release (mower_rs)"
  VERBATIM
)

foreach(_bin IN LISTS MOWER_RS_BINARIES)
  install(PROGRAMS "${_cargo_target_dir}/release/${_bin}"
    DESTINATION lib/${PROJECT_NAME})
endforeach()

ament_package()
