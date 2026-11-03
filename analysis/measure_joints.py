import json
from OCP.TopExp import TopExp_Explorer
from OCP.TopAbs import TopAbs_FACE, TopAbs_EDGE
from OCP.TopoDS import TopoDS
from OCP.BRepAdaptor import BRepAdaptor_Surface, BRepAdaptor_Curve
from OCP.GeomAbs import GeomAbs_Cylinder, GeomAbs_Circle
from OCP.collections import Sequence_TDF_Label
from measure_frame import BASE, OUT, load, name, bounds, labels

doc, tool = load(BASE/'三维STEP通用格式'/'Lm2六足机器人完整体.STEP')
seq=Sequence_TDF_Label()
tool.GetShapes(seq)
selected={}
for label in labels(seq):
    n=name(label)
    if n not in ('总线双轴舵机','十字大u支架','小u连接件','腿部支架','脚部支架'):
        continue
    shape=tool.GetShape_s(label)
    surfaces=[]
    ex=TopExp_Explorer(shape,TopAbs_FACE)
    while ex.More():
        ad=BRepAdaptor_Surface(TopoDS.Face(ex.Current()))
        if ad.GetType()==GeomAbs_Cylinder:
            cy=ad.Cylinder()
            p,d=cy.Location(),cy.Axis().Direction()
            surfaces.append({'r':cy.Radius(),'p':[p.X(),p.Y(),p.Z()],'d':[d.X(),d.Y(),d.Z()]})
        ex.Next()
    circles=[]
    ex=TopExp_Explorer(shape,TopAbs_EDGE)
    while ex.More():
        ad=BRepAdaptor_Curve(TopoDS.Edge(ex.Current()))
        if ad.GetType()==GeomAbs_Circle:
            cy=ad.Circle()
            p,d=cy.Location(),cy.Axis().Direction()
            circles.append({'r':cy.Radius(),'p':[p.X(),p.Y(),p.Z()],'d':[d.X(),d.Y(),d.Z()]})
        ex.Next()
    def unique(rows):
        return list({tuple(round(v,5) for v in [r['r']]+r['p']+r['d']):r for r in rows}.values())
    selected[n]={'bounds':bounds(shape),'cylinders':unique(surfaces),'circles':unique(circles)}
    print(n,json.dumps(selected[n],ensure_ascii=False))
(OUT/'Lm2_joint_geometry.json').write_text(json.dumps(selected,ensure_ascii=False,indent=2),encoding='utf-8')
