# 2026-09-01T09:31:28.580355100
import vitis

client = vitis.create_client()
client.set_workspace(path="VitisWS")

platform = client.get_component(name="AXI_Smart_Sorter")
status = platform.build()

status = platform.build()

comp = client.get_component(name="hello_world_AXI_Smart_Sorter")
comp.build()

status = platform.build()

comp.build()

status = platform.build()

comp.build()

