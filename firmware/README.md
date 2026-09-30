# Firmware (Zephyr) - not started

Planned for milestone M1. Layout when created:

```
firmware/
  west.yml            # pins Zephyr version
  app/                # Zephyr application: fc (flight computer) and act (voter) images
    CMakeLists.txt
    prj.conf
    boards/nucleo_g474re.overlay
    src/
```
`core/` is consumed as-is; only drivers, threads and ISRs live here.
