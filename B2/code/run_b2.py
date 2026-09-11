"""JSON CLI and reproducible SVG; no third-party runtime dependencies."""
import argparse
import hashlib
from html import escape
import json
import math
from pathlib import Path
import platform
from b2_solver import solve_b2
from domains import Problem


def _b1_geometry_path():
    project_root = Path(__file__).resolve().parents[2]
    for path in (project_root / "B1" / "code" / "b1_geometry.py",
                 project_root / "问题一" / "code" / "b1_geometry.py"):
        if path.is_file():
            return path
    raise FileNotFoundError("B1/code/b1_geometry.py not found")


def render_svg(result):
    head='<svg xmlns="http://www.w3.org/2000/svg" width="1200" height="900" viewBox="0 0 1200 900">'
    parts=[head,'<rect width="1200" height="900" fill="white"/>',
           '<style>text{font-family:"Microsoft YaHei",Arial,sans-serif;fill:#243746} .small{font-size:14px}</style>']
    def text(x,y,value,size=16,color=None):
        parts.append(f'<text x="{x}" y="{y}" font-size="{size}"'+(f' style="fill:{color}"' if color else '')+'>'+escape(str(value))+'</text>')
    text(45,40,'B2  第二检测点：接收保障与最坏情况定位',25)
    text(45,68,'数据标记：'+result.get('data_kind','unspecified')+'  |  主指标 D，辅助指标 R_min；非连续全局最优证明',15)
    if not result.get('recommended'):
        text(45,130,'状态：'+result['status']);parts.append('</svg>');return '\n'.join(parts)
    p=Problem(result['observation'],result['error_deg'],**result['physical_parameters'])
    scale=0.29*1000/p.rmin
    def xy(q): return (110+q[0]*scale,455-q[1]*scale)
    def circle(q,r,style):
        x,y=xy(q);return f'<circle cx="{x:.5f}" cy="{y:.5f}" r="{r*scale:.5f}" {style}/>'
    parts.append('<defs><clipPath id="plot"><rect x="55" y="120" width="650" height="650"/></clipPath>')
    c,s=math.cos(math.radians(p.alpha)),math.sin(math.radians(p.alpha))
    for name,q,r in [('c1',(0,0),p.rmin),('c2',(p.rmin*c,p.rmin*s),p.rmin),
                     ('c3',(p.rmin*c,-p.rmin*s),p.rmin),('src',(0,0),p.rmax),('d0',p.center,p.radius)]:
        parts.append(f'<clipPath id="{name}">'+circle(q,r,'')+'</clipPath>')
    parts.append('</defs><g clip-path="url(#plot)">')
    parts.append('<g clip-path="url(#c1)"><g clip-path="url(#c2)"><g clip-path="url(#c3)"><rect x="55" y="120" width="650" height="650" fill="#e7edf1"/></g></g></g>')
    tri=[(0,0),(p.rmax,p.rmax*math.tan(math.radians(p.alpha))),(p.rmax,-p.rmax*math.tan(math.radians(p.alpha)))]
    pts=' '.join(f'{x:.4f},{y:.4f}' for x,y in map(xy,tri))
    parts.append(f'<g clip-path="url(#src)"><g clip-path="url(#d0)"><polygon points="{pts}" fill="#d79322"/></g></g>')
    for i in range(1,31):
        for j in range(-30,31):
            q=(i*p.rmin/30,j*p.rmin/30)
            if p.effective_domain(q):
                x,y=xy(q);parts.append(f'<circle cx="{x:.3f}" cy="{y:.3f}" r="1.3" fill="#aabdc7"/>')
    preferred={tuple(q) for q in result['preferred_candidates_local_m']}
    for item in result['candidates']:
        x,y=xy(item['point_local_m'])
        if tuple(item['point_local_m']) in preferred:
            parts.append(f'<rect x="{x-3:.3f}" y="{y-3:.3f}" width="6" height="6" fill="#198575"/>')
        else: parts.append(f'<circle cx="{x:.3f}" cy="{y:.3f}" r="2.5" fill="#3577a3"/>')
    best=result['recommended'];bx,by=xy(best['point_local_m'])
    parts.append(f'<path d="M {bx-8},{by} H {bx+8} M {bx},{by-8} V {by+8}" stroke="#b23d3d" stroke-width="3"/>')
    parts.append('</g>')
    parts.append('<path d="M 65,455 H 690 M 110,130 V 760" stroke="#6b7c89" stroke-width="1"/>')
    for v in (0,500,1000,1500):
        x,y=xy((v*p.rmin/1000,0));text(x-10,y+22,int(v*p.rmin/1000),12)
    for v in (-1000,-500,500,1000):
        x,y=xy((0,v*p.rmin/1000));text(60,y+4,int(v*p.rmin/1000),12)
    text(390,495,'沿首次示向 a (m)',14);text(70,110,'横向 b (m)',14)
    text(160,785,'灰底 C_cert；浅点 C0_sub；蓝点已评估；绿方块优选点集',14)
    text(160,810,'橙色 H 为源集合 K 的凸外包络；红十字为推荐点',14)
    text(160,835,'C_safe 采用隐式定义，完整边界未绘制。',14)

    text(745,112,'最坏直径采样读数下的定位区域',18)
    geom=best['worst_diameter']['sample_geometry_local'];vertices=geom['vertices'];center=geom['mec_center'];r=geom['mec_radius_m']
    ss=330/max(2*r,1e-8)
    def qq(q): return (950+(q[0]-center[0])*ss,330-(q[1]-center[1])*ss)
    poly=' '.join(f'{x:.4f},{y:.4f}' for x,y in map(qq,vertices))
    parts.append(f'<circle cx="950" cy="330" r="{r*ss}" fill="none" stroke="#b23d3d" stroke-width="2"/>')
    parts.append(f'<polygon points="{poly}" fill="#d7e7f1" stroke="#3577a3" stroke-width="2"/>')
    parts.append('<circle cx="950" cy="330" r="3" fill="#b23d3d"/>')
    text(755,525,'等比例局部放大；红圆为该读数的最小覆盖圆',14)
    d=best['worst_diameter'];r=best['worst_mec_radius']
    text(745,575,f"推荐点 (a,b)：({best['point_local_m'][0]:.1f}, {best['point_local_m'][1]:.1f}) m")
    text(745,610,f"最坏 D：[{d['sample_lower_m']:.3f}, {d['envelope_upper_m']:.3f}] m")
    text(745,645,f"最坏 R_min：[{r['sample_lower_m']:.3f}, {r['envelope_upper_m']:.3f}] m")
    text(745,680,f"路程：{best['movement_m']:.3f} m；高精度复核：{best['high_precision_check']['status']}",14)
    text(745,715,'数值包络区间，未采用严格外向舍入。',14)
    text(745,746,'两个指标的最坏读数分别搜索。',14)
    text(45,878,'来源：同名结果 JSON；候选区域示意不代表完整保证接收域或连续近优域。',14)
    parts.append('</svg>');return '\n'.join(parts)


def run_file(input_path,output_dir):
    input_path=Path(input_path).resolve();output_dir=Path(output_dir).resolve()
    data=json.loads(input_path.read_text(encoding='utf-8-sig'))
    if not isinstance(data,dict) or set(data)-{'observation','error_deg','options','physical','data_kind','description'}:
        raise ValueError('invalid root JSON fields')
    if 'observation' not in data: raise ValueError('missing observation')
    target=output_dir/(input_path.stem+'.json')
    if target==input_path: raise ValueError('output must not overwrite input JSON')
    result=solve_b2(data['observation'],data.get('error_deg',1),options=data.get('options'),physical=data.get('physical'))
    result['data_kind']=data.get('data_kind','unspecified')
    result['provenance']={'input_name':input_path.name,'input_sha256':hashlib.sha256(input_path.read_bytes()).hexdigest(),
                          'python':platform.python_version(),'search_randomness':'none; deterministic',
                          'code_sha256':{q.name:hashlib.sha256(q.read_bytes()).hexdigest() for q in sorted(Path(__file__).parent.glob('*.py'))},
                          'b1_dependency_sha256':hashlib.sha256(_b1_geometry_path().read_bytes()).hexdigest()}
    output_dir.mkdir(parents=True,exist_ok=True)
    target.write_text(json.dumps(result,ensure_ascii=False,indent=2,allow_nan=False)+'\n',encoding='utf-8')
    (output_dir/(input_path.stem+'.svg')).write_text(render_svg(result),encoding='utf-8')
    return result


def main():
    parser=argparse.ArgumentParser(description='B2 conservative minimax second-station planner')
    parser.add_argument('input',type=Path)
    parser.add_argument('--output-dir',type=Path,default=Path(__file__).resolve().parents[1]/'结果')
    args=parser.parse_args()
    try: result=run_file(args.input,args.output_dir)
    except (ValueError,TypeError,KeyError,OSError) as exc: parser.error(str(exc))
    best=result.get('recommended')
    print(json.dumps({'status':result['status'],'recommended_world_m':best['point_world_m'] if best else None,
                      'worst_diameter_bounds_m':[best['worst_diameter'][k] for k in ('sample_lower_m','envelope_upper_m')] if best else None},ensure_ascii=False))


if __name__=='__main__': main()
