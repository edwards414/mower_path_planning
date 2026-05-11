cmake_minimum_required(VERSION 3.8)
project(mower_coverage_core)

find_package(ament_cmake REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter)

# Build the maturin wheel at CMake configure time
execute_process(
  COMMAND ${Python3_EXECUTABLE} -m maturin build --release
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

# Install the wheel via pip at install time
install(CODE
  "execute_process(
    COMMAND ${Python3_EXECUTABLE} -m pip install --no-deps --prefix
            \"${CMAKE_INSTALL_PREFIX}\"
            \"${CMAKE_CURRENT_BINARY_DIR}/wheels\"
    RESULT_VARIABLE _pip_result
  )
  if(NOT _pip_result EQUAL 0)
    message(FATAL_ERROR \"pip install of mower_coverage_core wheel failed\")
  endif()"
)

ament_package()
