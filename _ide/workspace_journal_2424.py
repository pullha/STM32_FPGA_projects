# 2026-08-20T09:47:43.622629100
import vitis

client = vitis.create_client()
client.set_workspace(path="VitisWS")

platform = client.create_platform_component(name = "platform_FND",hw_design = "C:\Users\X\vivado\project_1_FND\design_1_FND_wrapper.xsa",os = "standalone",cpu = "microblaze_riscv_0",domain_name = "standalone_microblaze_riscv_0")

vitis.dispose()

