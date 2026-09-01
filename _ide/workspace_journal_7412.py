# 2026-08-21T10:05:11.153138400
import vitis

client = vitis.create_client()
client.set_workspace(path="VitisWS")

platform = client.create_platform_component(name = "platform_Timer",hw_design = "C:\Users\X\vivado\project_1_Timer\design_1_Timer_wrapper.xsa",os = "standalone",cpu = "microblaze_riscv_0",domain_name = "standalone_microblaze_riscv_0")

comp = client.create_app_component(name="hello_world_Timer",platform = "$COMPONENT_LOCATION/../platform_Timer/export/platform_Timer/platform_Timer.xpfm",domain = "standalone_microblaze_riscv_0",template = "hello_world")

comp = client.get_component(name="hello_world_Timer")
status = comp.clean()

status = comp.clean()

status = comp.clean()

platform = client.get_component(name="platform_Timer")
status = platform.build()

status = comp.clean()

status = platform.build()

status = comp.clean()

status = comp.clean()

comp.build()

status = platform.build()

comp.build()

status = platform.build()

status = platform.build()

status = platform.update_desc(desc="")

status = comp.clean()

status = comp.clean()

status = platform.build()

status = comp.clean()

status = comp.clean()

status = comp.clean()

status = platform.build()

status = platform.build()

comp.build()

status = comp.clean()

status = comp.clean()

status = comp.clean()

status = comp.clean()

status = comp.clean()

status = platform.build()

comp.build()

platform = client.create_platform_component(name = "platform_PWM",hw_design = "C:\Users\X\vivado\project_1_PWM\design_1_PWM_wrapper.xsa",os = "standalone",cpu = "microblaze_riscv_0",domain_name = "standalone_microblaze_riscv_0")

comp = client.create_app_component(name="hello_world_PWM",platform = "$COMPONENT_LOCATION/../platform_PWM/export/platform_PWM/platform_PWM.xpfm",domain = "standalone_microblaze_riscv_0",template = "hello_world")

platform = client.get_component(name="platform_PWM")
status = platform.build()

comp = client.get_component(name="hello_world_PWM")
status = comp.clean()

status = platform.build()

comp.build()

