import bpy
from BlenderMavlink import BlenderMavlink
import time

scene = bpy.context.scene


"""
def menu_func(self, context):
    self.layout.operator(BlenderMavlink.BlenderMavlinkOperator.bl_idname, text="Hello World Operator")

bpy.utils.register_class(BlenderMavlink.BlenderMavlinkOperator)
bpy.types.VIEW3D_MT_view.append(menu_func)

bpy.ops.object.modal_operator('INVOKE_DEFAULT', scene, 'udp:localhost:14550')
"""

b = BlenderMavlink.BlenderMavlink(scene=scene, port="udp:localhost:14550")
b.run()
time.sleep(3)
b.exit()
