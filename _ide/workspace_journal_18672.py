# 2026-08-20T14:32:32.580072500
import vitis

client = vitis.create_client()
client.set_workspace(path="VitisWS")

platform = client.create_platform_component(name = "platform_URAT_INT",hw_design = "C:\Users\X\vivado\project_1_UART_INT\design_1_UART_INT_wrapper.xsa",os = "standalone",cpu = "microblaze_riscv_0",domain_name = "standalone_microblaze_riscv_0")

comp = client.create_app_component(name="hello_world_UART_INT",platform = "$COMPONENT_LOCATION/../platform_URAT_INT/export/platform_URAT_INT/platform_URAT_INT.xpfm",domain = "standalone_microblaze_riscv_0",template = "hello_world")

comp = client.get_component(name="hello_world_UART_INT")
status = comp.clean()

platform = client.get_component(name="platform_URAT_INT")
status = platform.build()

comp.build()

vitis.dispose()

vitis.dispose()

