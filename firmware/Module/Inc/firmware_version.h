/*
 * firmware_version.h
 *
 * Build identity reported over UART in the 0x87 FIRMWARE_INFO frame so the
 * host (firmware-sync at container start, the ros2_control driver, the app)
 * can tell which build is running.
 *
 * The values come from the build system: `make` at the firmware root derives
 * them from `git describe` (or from MOWER_VERSION / MOWER_GIT_SHA passed by
 * the Docker build). A CubeIDE build that defines none of them reports
 * 0.0.0 / sha 0 / FW_BUILD_FLAG_UNVERSIONED, which firmware-sync treats as
 * "unknown build, replace with the bundled one".
 */
#ifndef INC_FIRMWARE_VERSION_H_
#define INC_FIRMWARE_VERSION_H_

#include <stdint.h>

#ifndef FW_VERSION_MAJOR
#define FW_VERSION_MAJOR 0U
#endif
#ifndef FW_VERSION_MINOR
#define FW_VERSION_MINOR 0U
#endif
#ifndef FW_VERSION_PATCH
#define FW_VERSION_PATCH 0U
#endif
/* First 4 bytes of the commit hash as a big-endian number (0x12345678 for
 * commit 12345678...). 0 when unknown. */
#ifndef FW_GIT_SHA
#define FW_GIT_SHA 0x00000000UL
#endif
/* Unix time of the build, seconds. Distinguishes rebuilds of a dirty tree. */
#ifndef FW_BUILD_UNIX
#define FW_BUILD_UNIX 0UL
#endif
#ifndef FW_BUILD_FLAGS
#define FW_BUILD_FLAGS FW_BUILD_FLAG_UNVERSIONED
#endif

/* FW_BUILD_FLAGS bits */
#define FW_BUILD_FLAG_DIRTY 0x01U        /* built from a tree with local changes */
#define FW_BUILD_FLAG_UNVERSIONED 0x02U  /* build system passed no version */

#endif /* INC_FIRMWARE_VERSION_H_ */
