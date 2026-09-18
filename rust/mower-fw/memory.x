/* STM32F411CE, laid out to match Module/Inc/boot_shared.h.
 *
 *   0x08000000  32 KB   bootloader (sectors 0-1)     -- not ours
 *   0x08008000  352 KB  application (sectors 2-6)    -- this image
 *   0x08060000  128 KB  settings (sector 7)          -- mower_core::settings
 *
 *   0x20000000  32 B    reset-surviving boot mailbox -- BOOT_SHARED
 *   0x20000020  128 KB - 32  RAM
 */
MEMORY
{
  FLASH : ORIGIN = 0x08008000, LENGTH = 352K
  RAM   : ORIGIN = 0x20000020, LENGTH = 128K - 32
}
