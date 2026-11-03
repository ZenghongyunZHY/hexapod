from pathlib import Path
import collections
import json
import re
import tempfile

from OCP.STEPCAFControl import STEPCAFControl_Reader
from OCP.TDocStd import TDocStd_Document
from OCP.TCollection import TCollection_ExtendedString
from OCP.TDataStd import TDataStd_Name
from OCP.TDF import TDF_Label
from OCP.collections import Sequence_TDF_Label as TDF_LabelSequence
from OCP.XCAFDoc import XCAFDoc_DocumentTool
from OCP.Bnd import Bnd_Box
from OCP.BRepBndLib import BRepBndLib
from OCP.IFSelect import IFSelect_RetDone

BASE = Path(r'C:\baidunetdiskdownload\021 Lm2六足机器人图纸\021 Lm2六足机器人图纸')
OUT = Path(__file__).resolve().parents[1] / 'outputs'
OUT.mkdir(exist_ok=True)

def name(label):
    attr = TDataStd_Name()
    return attr.Get().ToExtString() if label.FindAttribute(TDataStd_Name.GetID_s(), attr) else ''

def bounds(shape):
    if shape.IsNull():
        return None
    box = Bnd_Box()
    BRepBndLib.AddOptimal_s(shape, box, False, False)
    lo, hi = box.CornerMin(), box.CornerMax()
    b = (lo.X(), lo.Y(), lo.Z(), hi.X(), hi.Y(), hi.Z())
    return {'min': list(b[:3]), 'max': list(b[3:]), 'size': [b[i+3]-b[i] for i in range(3)]}

def load(path):
    raw = path.read_bytes()
    try:
        text = raw.decode('utf-8')
    except UnicodeDecodeError:
        text = raw.decode('gb18030')
    text = re.sub(r'[^\x00-\x7f]+', lambda m: '\\X2\\'+m[0].encode('utf-16-be').hex().upper()+'\\X0\\', text)
    doc = TDocStd_Document(TCollection_ExtendedString('MDTV-XCAF'))
    reader = STEPCAFControl_Reader()
    reader.SetNameMode(True)
    with tempfile.TemporaryDirectory() as directory:
        normalized = Path(directory) / 'model.step'
        normalized.write_text(text, encoding='ascii')
        assert reader.ReadFile(str(normalized)) == IFSelect_RetDone
        assert reader.Transfer(doc)
    tool = XCAFDoc_DocumentTool.ShapeTool_s(doc.Main())
    return doc, tool

def labels(seq):
    return [seq.Value(i) for i in range(1, seq.Length()+1)]

def measure(path):
    doc, tool = load(path)
    roots = TDF_LabelSequence()
    tool.GetFreeShapes(roots)
    all_labels = TDF_LabelSequence()
    tool.GetShapes(all_labels)
    definitions = []
    for label in labels(all_labels):
        definitions.append({'name': name(label), 'assembly': tool.IsAssembly_s(label), 'bounds': bounds(tool.GetShape_s(label))})
    tree = []
    def walk(label, path, location):
        referred = TDF_Label()
        if tool.IsReference_s(label):
            assert tool.GetReferredShape_s(label, referred)
            location = location.Multiplied(tool.GetLocation_s(label))
            label = referred
        current = path + [name(label)]
        shape = tool.GetShape_s(label)
        if not shape.IsNull():
            shape = shape.Moved(location)
        tree.append({'path': current, 'assembly': tool.IsAssembly_s(label), 'bounds': bounds(shape)})
        children = TDF_LabelSequence()
        if tool.GetComponents_s(label, children, False):
            for child in labels(children):
                walk(child, current, location)
    from OCP.TopLoc import TopLoc_Location
    for root in labels(roots):
        walk(root, [], TopLoc_Location())
    return {'file': str(path), 'units': 'mm', 'definitions': definitions, 'instances': tree}

if __name__ == '__main__':
    paths = [BASE/'三维STEP通用格式'/'Lm2六足机器人完整体.STEP', BASE/'三维SolidWorks格式'/'Lm2六足支架'/'Lm2六足底板.STEP']
    results = [measure(p) for p in paths]
    (OUT/'Lm2_model_measurements.json').write_text(json.dumps(results, ensure_ascii=False, indent=2), encoding='utf-8')
    for result in results:
        print(result['file'])
        for row in result['definitions']:
            print(json.dumps(row, ensure_ascii=False))
        print('TOTAL INSTANCES', len(result['instances']))
