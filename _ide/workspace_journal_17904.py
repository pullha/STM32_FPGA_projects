# 2026-08-25T16:35:20.993754900
import vitis

client = vitis.create_client()
client.set_workspace(path="VitisWS")

platform = client.create_platform_component(name = "AXI_Smart_Sorter",hw_design = "C:\Users\X\vivado\AXI_Smart_Sorter\AXI_Smart_Sorter_wrapper.xsa",os = "standalone",cpu = "ps7_cortexa9_0",domain_name = "standalone_ps7_cortexa9_0")

platform = client.get_component(name="AXI_Smart_Sorter")
status = platform.build()

comp = client.create_app_component(name="hello_world_AXI_Smart_Sorter",platform = "$COMPONENT_LOCATION/../AXI_Smart_Sorter/export/AXI_Smart_Sorter/AXI_Smart_Sorter.xpfm",domain = "standalone_ps7_cortexa9_0",template = "hello_world")

comp = client.get_component(name="hello_world_AXI_Smart_Sorter")
status = comp.clean()

status = comp.clean()

status = platform.build()

comp.build()

vitis.dispose()

