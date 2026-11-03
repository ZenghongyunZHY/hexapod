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
from OCP.collections import Sequence_TDF_Label
from OCP.TDF import TDF_Label
from OCP.TopLoc import TopLoc_Location
from measure_frame import BASE, OUT, load, name, labels, bounds

doc,tool=load(BASE/'三维STEP通用格式'/'Lm2六足机器人完整体.STEP')
seq=Sequence_TDF_Label(); tool.GetShapes(seq)
parts={name(l):tool.GetShape_s(l) for l in labels(seq)}
def edges(shape,a,b):
    ex=TopExp_Explorer(shape,TopAbs_EDGE); lines=[]
    while ex.More():
        c=BRepAdaptor_Curve(TopoDS.Edge(ex.Current()))
        ts=np.linspace(c.FirstParameter(),c.LastParameter(),2 if c.GetType()==GeomAbs_Line else 25)
        ps=[c.Value(float(t)) for t in ts]
        lines.append([([p.X(),p.Y(),p.Z()][a],[p.X(),p.Y(),p.Z()][b]) for p in ps])
        ex.Next()
    return lines
plt.rcParams['font.family']='Microsoft YaHei'
fig,axs=plt.subplots(2,2,figsize=(12,10))
for ax,(n,a,b) in zip(axs.flat,[('总线双轴舵机',1,2),('十字大u支架',1,2),('小u连接件',2,1),('腿部支架',0,2)]):
    ax.add_collection(LineCollection(edges(parts[n],a,b),colors='#334155',linewidths=.65))
    ax.autoscale(); ax.set_aspect('equal'); ax.set_title(n); ax.grid(alpha=.2)
fig.tight_layout(); fig.savefig(OUT/'Lm2_腿长孔位核对.png',dpi=150)

# The left leg's local coordinates put its long direction along -Y.
leg=parts['左腿']
fig,ax=plt.subplots(figsize=(6,12))
ax.add_collection(LineCollection(edges(leg,2,1),colors='#334155',linewidths=.55))
ax.autoscale();ax.set_aspect('equal');ax.grid(alpha=.2)
ax.set_title('左腿装配模型 · 展开姿态')
fig.tight_layout();fig.savefig(OUT/'Lm2_左腿模型核对.png',dpi=140)

# Walk one left leg using its own local frame; print each relevant part's transformed circles.
all_labels={name(l):l for l in labels(seq)}
from OCP.GeomAbs import GeomAbs_Circle
import json
rows=[]
def walk(label,path,loc):
    if tool.IsReference_s(label):
        ref=TDF_Label();tool.GetReferredShape_s(label,ref)
        loc=loc.Multiplied(tool.GetLocation_s(label));label=ref
    n=name(label);current=path+[n]
    shape=tool.GetShape_s(label).Moved(loc)
    if n in ('总线双轴舵机','十字大u支架','腿部支架','脚部支架','脚垫'):
        row={'path':current,'bounds':bounds(shape),'circles':[]}
        ex=TopExp_Explorer(shape,TopAbs_EDGE)
        while ex.More():
            c=BRepAdaptor_Curve(TopoDS.Edge(ex.Current()))
            if c.GetType()==GeomAbs_Circle and c.Circle().Radius()>=2.8:
                cc=c.Circle();p=cc.Location();d=cc.Axis().Direction()
                row['circles'].append({'r':cc.Radius(),'p':[p.X(),p.Y(),p.Z()],'d':[d.X(),d.Y(),d.Z()]})
            ex.Next()
        rows.append(row)
    cs=Sequence_TDF_Label()
    if tool.GetComponents_s(label,cs,False):
        for child in labels(cs):walk(child,current,loc)
walk(all_labels['左腿'],[],TopLoc_Location())
(OUT/'Lm2_left_leg_axes.json').write_text(json.dumps(rows,ensure_ascii=False,indent=2),encoding='utf-8')
for r in rows:
    print('/'.join(r['path']),r['bounds'])
    if r['path'][-1]=='总线双轴舵机':
        print('shaft',[[round(t,4) for t in c['p']] for c in r['circles'] if 2.8<c['r']<3.5][:5])
    if r['path'][-1]=='腿部支架':
        print('hole',r['circles'])
