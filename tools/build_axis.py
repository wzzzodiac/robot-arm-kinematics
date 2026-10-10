"""AXIS planar rig. Blender XY = solver XY; Blender Z = presentation height.
Run: blender --background --python tools/build_axis.py --python-exit-code 1
Link bodies scale in local X, while the next pivot follows the requested length.
"""
from pathlib import Path
from math import pi, sin, cos
import bpy
ROOT=Path(__file__).resolve().parents[1]
bpy.ops.object.select_all(action='SELECT'); bpy.ops.object.delete(use_global=False)
def material(name,color,metal=0,rough=.4):
 m=bpy.data.materials.new(name);m.diffuse_color=(*color,1);m.use_nodes=True
 b=m.node_tree.nodes.get('Principled BSDF');b.inputs['Base Color'].default_value=(*color,1);b.inputs['Metallic'].default_value=metal;b.inputs['Roughness'].default_value=rough
 return m
ivory=material('Porcelain enamel',(.78,.79,.73),.22,.3)
orange=material('Signal orange',(.95,.17,.035),.25,.32)
dark=material('Graphite anodized',(.045,.055,.059),.65,.32)
steel=material('Brushed titanium',(.38,.44,.46),.85,.28)
rubber=material('Soft rubber',(.015,.021,.023),0,.8)
def empty(name,parent=None,loc=(0,0,0)):
 o=bpy.data.objects.new(name,None);bpy.context.collection.objects.link(o);o.parent=parent;o.location=loc;return o
root=empty('AxisRoot')
def finish(o,name,parent,mat):
 o.name=name;o.data.materials.append(mat);o.parent=parent
 bevel=o.modifiers.new('Machined bevel','BEVEL');bevel.width=1.4;bevel.segments=3
 o.modifiers.new('Surface normals','WEIGHTED_NORMAL');return o
def box(name,parent,loc,size,mat):
 bpy.ops.mesh.primitive_cube_add(size=1);o=bpy.context.object;o.dimensions=size;bpy.ops.object.transform_apply(location=False,rotation=False,scale=True);finish(o,name,parent,mat);o.location=loc;return o
def cyl(name,parent,loc,r,d,mat,vertices=48):
 bpy.ops.mesh.primitive_cylinder_add(vertices=vertices,radius=r,depth=d);o=bpy.context.object;finish(o,name,parent,mat);o.location=loc;return o
def bolts(parent,r,z,count=6):
 for i in range(count):
  a=i*2*pi/count;cyl('SocketBolt',parent,(cos(a)*r,sin(a)*r,z),2.3,2,steel,6)
box('Mount',root,(0,0,7),(98,98,14),dark)
for x in [-37,37]:
 for y in [-37,37]:cyl('Anchor',root,(x,y,15),4,4,steel,6)
cyl('Pedestal',root,(0,0,39),34,52,ivory)
cyl('BaseRing',root,(0,0,22),36,7,orange)
cyl('Bearing',root,(0,0,64),29,8,steel)
j1=empty('J1',root,(0,0,70));j2=empty('J2',j1,(180,0,16));j3=empty('J3',j2,(150,0,16))
for n,j,length,width in [(1,j1,180,31),(2,j2,150,27),(3,j3,75,22)]:
 body=empty(f'Body{n}',j)
 box(f'Link{n}Shell',body,(length/2,0,0),(length,width,18),ivory)
 box(f'Link{n}Underlay',body,(length/2,0,-8),(length-8,width-5,6),dark)
 box(f'Link{n}Stripe',body,(length*.52,-width/2-.25,1),(length*.52,1.5,8),orange)
 box(f'Link{n}TopPanel',body,(length*.53,0,10),(length*.60,width*.61,3),dark)
 for i in range(5):box('CoolingSlot',body,(length*.31+i*7,0,12),(2,width*.39,1),steel)
 cyl(f'Joint{n}Housing',j,(0,0,0),width*.75,22,ivory)
 cyl(f'Joint{n}Seal',j,(0,0,12),width*.66,4,dark)
 cyl(f'Joint{n}Cover',j,(0,0,15),width*.55,3,steel)
 cyl(f'Joint{n}Badge',j,(0,0,18),width*.28,3,orange)
 bolts(j,width*.44,18)
tcp=empty('TCP',j3,(75,0,0))
cyl('ToolFlange',tcp,(0,0,-8),14,20,steel)
box('GripperHousing',tcp,(6,0,-28),(28,27,25),dark)
box('ToolCover',tcp,(7,0,-14),(29,28,4),orange)
for name,y in [('FingerL',17),('FingerR',-17)]:
 f=empty(name,tcp,(0,y,0));box('FingerRail',f,(7,0,-40),(29,5,9),steel);box('FingerTip',f,(22,0,-46),(6,7,20),rubber)
bpy.context.view_layer.update()
bpy.ops.object.select_all(action='DESELECT')
for o in [root,*root.children_recursive]:o.select_set(True)
bpy.context.view_layer.objects.active=root
bpy.ops.wm.save_as_mainfile(filepath=str(ROOT/'assets/axis-arm.blend'))
bpy.ops.export_scene.gltf(filepath=str(ROOT/'assets/axis-arm.glb'),export_format='GLB',use_selection=True,export_apply=True)
print('AXIS: exported AxisRoot > J1 > J2 > J3 > TCP, Body1/2/3, FingerL/R')
