# 2026-08-24T10:24:41.803497500
import vitis

client = vitis.create_client()
client.set_workspace(path="VitisWS")

platform = client.create_platform_component(name = "platform_MYIP_FND",hw_design = "C:\Users\X\vivado\project_1_MYIP_FND\design_1_MYIP_FND_wrapper.xsa",os = "standalone",cpu = "microblaze_riscv_0",domain_name = "standalone_microblaze_riscv_0")

comp = client.create_app_component(name="hello_world_MYIP_FND",platform = "$COMPONENT_LOCATION/../platform_MYIP_FND/export/platform_MYIP_FND/platform_MYIP_FND.xpfm",domain = "standalone_microblaze_riscv_0",template = "hello_world")

platform = client.get_component(name="platform_MYIP_FND")
status = platform.build()

platform = client.get_component(name="platform_URAT_INT")
status = platform.build()

comp = client.get_component(name="hello_world_UART_INT")
comp.build()

platform = client.get_component(name="platform_MYIP_FND")
status = platform.build()

status = platform.build()

