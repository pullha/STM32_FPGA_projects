# 2026-08-19T12:01:02.946839
import vitis

client = vitis.create_client()
client.set_workspace(path="VitisWS")

platform = client.create_platform_component(name = "platform_LED",hw_design = "C:\Users\X\vivado\project_1_LED\design_1_LED_wrapper.xsa",os = "standalone",cpu = "microblaze_riscv_0",domain_name = "standalone_microblaze_riscv_0")

platform = client.get_component(name="platform_LED")
status = platform.update_desc(desc="")

comp = client.create_app_component(name="hello_world_LED",platform = "$COMPONENT_LOCATION/../platform_LED/export/platform_LED/platform_LED.xpfm",domain = "standalone_microblaze_riscv_0",template = "hello_world")

status = platform.build()

comp = client.get_component(name="hello_world_LED")
comp.build()

component = client.get_component(name="hello_world_LED")

lscript = component.get_ld_script(path="C:\FPGA\VitisWS\hello_world_LED\src\lscript.ld")

lscript.regenerate()

status = platform.build()

comp.build()

status = platform.build()

comp.build()

status = platform.build()

comp.build()

status = platform.build()

comp.build()

status = platform.build()

comp.build()

status = platform.build()

comp.build()

