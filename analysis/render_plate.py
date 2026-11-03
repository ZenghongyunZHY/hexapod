from pathlib import Path
import json
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection
from OCP.TopExp import TopExp_Explorer
from OCP.TopAbs import TopAbs_EDGE
from OCP.TopoDS import TopoDS
from OCP.BRepAdaptor import BRepAdaptor_Curve
from OCP.GeomAbs import GeomAbs_Line
from OCP.TDF import TDF_Label
from OCP.collections import Sequence_TDF_Label
from measure_frame import BASE, OUT, load, bounds

plt.rcParams['font.family'] = 'Microsoft YaHei'
plt.rcParams['axes.unicode_minus'] = False
doc, tool = load(BASE/'三维SolidWorks格式'/'Lm2六足支架'/'Lm2六足底板.STEP')
roots = Sequence_TDF_Label()
tool.GetFreeShapes(roots)
shape = tool.GetShape_s(roots.Value(1))
b = bounds(shape)
explorer = TopExp_Explorer(shape, TopAbs_EDGE)
lines = []
while explorer.More():
    curve = BRepAdaptor_Curve(TopoDS.Edge(explorer.Current()))
    ts = np.linspace(curve.FirstParameter(), curve.LastParameter(), 2 if curve.GetType() == GeomAbs_Line else 40)
    pts = [curve.Value(float(t)) for t in ts]
    lines.append([(p.X()-b['min'][0], p.Z()-b['min'][2]) for p in pts])
    explorer.Next()
width, length = b['size'][0], b['size'][2]
fig, ax = plt.subplots(figsize=(7,9))
fig.patch.set_facecolor('#f8fafc')
ax.set_facecolor('#f8fafc')
ax.add_collection(LineCollection(lines, colors='#334155', linewidths=.75))
ax.annotate('', (0,-12), (width,-12), arrowprops={'arrowstyle':'<->','color':'#2563eb','lw':1.5})
ax.plot([0,0],[-16,0],color='#94a3b8',lw=.7)
ax.plot([width,width],[-16,0],color='#94a3b8',lw=.7)
ax.text(width/2,-20,f'{width:.2f} mm',ha='center',va='top',fontsize=13,color='#1d4ed8')
ax.annotate('', (width+12,0), (width+12,length), arrowprops={'arrowstyle':'<->','color':'#2563eb','lw':1.5})
ax.plot([width,width+16],[0,0],color='#94a3b8',lw=.7)
ax.plot([width,width+16],[length,length],color='#94a3b8',lw=.7)
ax.text(width+20,length/2,f'{length:.2f} mm',ha='center',va='center',rotation=90,fontsize=13,color='#1d4ed8')
ax.set_aspect('equal')
ax.set_xlim(-15,width+38)
ax.set_ylim(-35,length+13)
ax.axis('off')
ax.set_title('Lm2 底板 · 模型测量\n板厚 2.00 mm',fontsize=17,pad=18)
fig.text(.5,.035,'尺寸取自 STEP 实体；显示的是零件外廓尺寸。',ha='center',fontsize=10,color='#64748b')
fig.savefig(OUT/'Lm2_底板尺寸.png',dpi=180,bbox_inches='tight')
plt.close(fig)
print('Saved dimensioned plate preview')
