# 2026-08-20T09:56:59.478900900
import vitis

client = vitis.create_client()
client.set_workspace(path="VitisWS")

client.delete_component(name="platform_FND")

platform = client.create_platform_component(name = "platform_FND",hw_design = "C:\Users\X\vivado\project_1_FND\design_1_FND_wrapper.xsa",os = "standalone",cpu = "microblaze_riscv_0",domain_name = "standalone_microblaze_riscv_0")

platform = client.get_component(name="platform_FND")
status = platform.build()

comp = client.create_app_component(name="hello_world_FND",platform = "$COMPONENT_LOCATION/../platform_FND/export/platform_FND/platform_FND.xpfm",domain = "standalone_microblaze_riscv_0")

client.delete_component(name="hello_world_FND")

comp = client.create_app_component(name="hello_world_FND",platform = "$COMPONENT_LOCATION/../platform_FND/export/platform_FND/platform_FND.xpfm",domain = "standalone_microblaze_riscv_0",template = "hello_world")

status = platform.build()

comp = client.get_component(name="hello_world_FND")
comp.build()

status = comp.clean()

status = comp.clean()

status = comp.clean()

status = comp.clean()

status = comp.clean()

status = comp.clean()

status = platform.build()

comp.build()

status = comp.clean()

status = comp.clean()

status = comp.clean()

status = comp.clean()

status = comp.clean()

status = comp.clean()

platform = client.create_platform_component(name = "platform_LED_UART",hw_design = "C:\Users\X\vivado\project_1_LED_UART\design_1_LED_UART_wrapper.xsa",os = "standalone",cpu = "microblaze_riscv_0",domain_name = "standalone_microblaze_riscv_0")

comp = client.create_app_component(name="hello_world_LED_UART",platform = "$COMPONENT_LOCATION/../platform_LED_UART/export/platform_LED_UART/platform_LED_UART.xpfm",domain = "standalone_microblaze_riscv_0",template = "hello_world")

comp = client.get_component(name="hello_world_LED_UART")
status = comp.clean()

status = comp.clean()

platform = client.get_component(name="platform_LED_UART")
status = platform.build()

comp.build()

status = comp.clean()

status = comp.clean()

status = comp.clean()

status = comp.clean()

status = platform.build()

status = comp.clean()

comp.build()

vitis.dispose()

vitis.dispose()

