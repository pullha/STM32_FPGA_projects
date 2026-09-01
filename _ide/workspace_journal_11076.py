# 2026-08-28T12:15:47.686934500
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

status = platform.build()

comp.build()

status = platform.build()

comp.build()

status = platform.update_hw(hw_design = "C:\Users\X\vivado\AXI_Smart_Sorter\AXI_Smart_Sorter_wrapper.xsa")

status = platform.build()

status = platform.build()

comp.build()

status = platform.build()

status = platform.build()

comp.build()

vitis.dispose()

