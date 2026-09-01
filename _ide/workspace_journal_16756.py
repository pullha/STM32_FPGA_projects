# 2026-08-31T12:40:00.923396800
import vitis

client = vitis.create_client()
client.set_workspace(path="VitisWS")

platform = client.get_component(name="AXI_Smart_Sorter")
status = platform.update_hw(hw_design = "C:\Users\X\vivado\AXI_Smart_Sorter\AXI_Smart_Sorter_wrapper.xsa")

status = platform.build()

status = platform.build()

comp = client.get_component(name="hello_world_AXI_Smart_Sorter")
comp.build()

vitis.dispose()

