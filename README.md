# STM32 and FPGA Projects

This repository collects three embedded projects in separate category folders.

## Projects

- `smart-farm/` - Vivado FPGA smart-farm project originally stored as `PID_test`.
- `elevator/` - STM32F411 elevator controller project.
- `autonomous-exploration-vehicle/` - STM32F411 autonomous exploration vehicle project, including Raspberry Pi helper tools.
- `AXI_Smart_Sorter/` - Zynq SoC-based real-time sorting system using an OV7670 camera and custom AXI IP.

## Notes

Build outputs, simulation artifacts, generated run caches, zipped backups, and local credential folders are intentionally excluded. The STM32 projects keep their CubeMX/CMake source structure, while the FPGA project keeps the Vivado project file and source constraints/modules.
