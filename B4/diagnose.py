"""Print local B4 source fingerprints and module origins."""
import hashlib, importlib, json, platform, sys
from pathlib import Path
ROOT=Path(__file__).resolve().parent
REPOROOT=ROOT.parent
if str(REPOROOT) not in sys.path:
    sys.path.insert(0,str(REPOROOT))

def sha(path):
    text=path.read_text(encoding='utf-8').replace('\r\n','\n').replace('\r','\n')
    return hashlib.sha256(text.encode()).hexdigest()

manifest=json.loads((ROOT/'source_manifest.json').read_text(encoding='utf-8'))
print('python:',sys.version.replace('\n',' '))
print('platform:',platform.platform())
print('B4 root:',ROOT)
bad=[]
for rel,expected in manifest['sha256_lf'].items():
    p=ROOT/rel
    actual=sha(p) if p.is_file() else 'MISSING'
    ok=actual==expected
    print(('OK  ' if ok else 'BAD ')+rel+' '+actual)
    if not ok: bad.append(rel)
for name in ['B4.code.run_offline','B4.code.solver','B4.code.b4_geometry','B4.code.protocol','B4.code.vendor.b1plus']:
    m=importlib.import_module(name)
    print('MODULE',name,'=>',Path(m.__file__).resolve())
if bad:
    print('SOURCE_MISMATCHES:',bad)
    raise SystemExit(2)
print('source manifest: OK')
