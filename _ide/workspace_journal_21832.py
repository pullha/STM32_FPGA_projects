# 2026-08-27T12:02:12.815855600
import vitis

client = vitis.create_client()
client.set_workspace(path="VitisWS")

platform = client.get_component(name="AXI_Smart_Sorter")
status = platform.build()

status = platform.build()

comp = client.get_component(name="hello_world_AXI_Smart_Sorter")
comp.build()

status = platform.update_hw(hw_design = "C:\Users\X\vivado\AXI_Smart_Sorter\AXI_Smart_Sorter_wrapper.xsa")

status = platform.build()

comp.build()

vitis.dispose()

