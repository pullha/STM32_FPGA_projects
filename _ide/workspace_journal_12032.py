# 2026-08-20T15:30:02.127244800
import vitis

client = vitis.create_client()
client.set_workspace(path="VitisWS")

platform = client.get_component(name="platform_URAT_INT")
status = platform.build()

comp = client.get_component(name="hello_world_UART_INT")
comp.build()

