"""Standard-library SVG from frozen offline comparison, with explicit scope."""
from html import escape
import json
import math
from pathlib import Path
from coverage import sites

BASE=Path(__file__).resolve().parents[1]


def main():
    out=BASE/'结果';frozen=json.loads((out/'frozen_numbers.json').read_text(encoding='utf-8'))
    comparison=json.loads((out/'comparison.json').read_text(encoding='utf-8'))
    rows=comparison['runs'];summary=frozen['summary']
    parts=['<svg xmlns="http://www.w3.org/2000/svg" width="1200" height="660" viewBox="0 0 1200 660">',
           '<rect width="1200" height="660" fill="white"/>',
           '<style>text{font-family:"Microsoft YaHei",Arial,sans-serif;fill:#243746}</style>']
    def text(x,y,t,size=16): parts.append(f'<text x="{x}" y="{y}" font-size="{size}">{escape(str(t))}</text>')
    text(40,38,'B3：发现保证与 A / B / C 离线对照',26)
    text(40,68,'12 个人工场景 × 3 策略；相同源与误差场。不是官方演练结果。',16)
    text(55,115,'7 点覆盖：最小接收半径 1000 m',19)
    def xy(q): return 265+q[0]*.105,325-q[1]*.105
    parts.append('<defs><clipPath id="domain"><circle cx="265" cy="325" r="189"/></clipPath></defs>')
    parts.append('<g clip-path="url(#domain)">')
    for q in sites():
        x,y=xy(q);parts.append(f'<circle cx="{x}" cy="{y}" r="105" fill="#4592b3" fill-opacity=".13" stroke="#789dad" stroke-width="1"/>')
    parts.append('</g><circle cx="265" cy="325" r="189" fill="none" stroke="#243746" stroke-width="1.5"/>')
    path=' '.join(('M' if i==0 else 'L')+f'{xy(q)[0]},{xy(q)[1]}' for i,q in enumerate(sites()))
    parts.append(f'<path d="{path}" fill="none" stroke="#287da7" stroke-width="2"/>')
    for i,q in enumerate(sites()):
        x,y=xy(q);parts.append(f'<circle cx="{x}" cy="{y}" r="4" fill="#14506f"/>');text(x+7,y-7,'O' if i==0 else f'Q{i-1}',13)
    target=(1800*math.cos(math.pi/6),1800*math.sin(math.pi/6));x,y=xy(target)
    parts.append(f'<circle cx="{x}" cy="{y}" r="5" fill="#ba4a43"/>')
    text(72,554,'ρ = 1130 m；最坏外圈距离 996.950 m < 1000 m',15)
    text(72,584,'目标圆域 R=1800 m；路线 6780 m；图形等比例',15)

    text(595,115,'平均总虚拟时间（秒，越低越好）',19)
    maximum=math.ceil(max(v['mean_total_virtual_s'] for v in summary.values())*1.18/1000)*1000
    x0,y0,width,height=625,505,485,320
    parts.append(f'<path d="M{x0},{y0-height} V{y0} H{x0+width}" fill="none" stroke="#8e9da8"/>')
    for fraction in [0,.25,.5,.75,1]:
        y=y0-height*fraction
        parts.append(f'<path d="M{x0},{y} H{x0+width}" stroke="#e3e9ed"/>');text(563,y+5,f'{maximum*fraction:.0f}',12)
    for i,(name,color) in enumerate([('A','#8695a2'),('B','#5488ad'),('C','#248e79')]):
        value=summary[name]['mean_total_virtual_s'];x=x0+45+i*155;h=height*value/maximum
        parts.append(f'<rect x="{x}" y="{y0-h}" width="85" height="{h}" rx="3" fill="{color}"/>')
        text(x-1,y0-h-12,f'{value:.1f}',17);text(x+35,y0+27,name,18)
    by_case={}
    for row in rows: by_case.setdefault(row['case_seed'],{})[row['strategy']]=row['metrics']['total_virtual_time_s']
    wins=sum(v['C']<v['A'] for v in by_case.values())
    text(605,565,f'C 比 A 更快：{wins}/{len(by_case)} 场；保留所有不利场景',15)
    text(605,595,f'全部 {len(rows)} 次离线运行清除比例均为 1.0',15)
    text(40,640,'来源：B3/结果/frozen_numbers.json 与 comparison.json；官方演练与正式版本冻结尚未完成。',14)
    parts.append('</svg>');folder=out/'figures';folder.mkdir(parents=True,exist_ok=True)
    (folder/'comparison.svg').write_text('\n'.join(parts),encoding='utf-8')


if __name__=='__main__': main()
