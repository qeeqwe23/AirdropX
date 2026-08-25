from __future__ import annotations
from pathlib import Path
from datetime import datetime
import csv,json

def make_output_root(explicit=''):
    if explicit: p=Path(explicit)
    else: p=Path.home()/"Documents"/"AirdropX"/"results"/datetime.now().strftime('%Y%m%d_%H%M%S')
    p.mkdir(parents=True,exist_ok=True); return p

def save(root,config,series,cargo,summary):
    root=Path(root); config.save(root/'mission_config.json')
    if series:
        with (root/'timeseries.csv').open('w',newline='',encoding='utf-8-sig') as f:
            w=csv.DictWriter(f,fieldnames=list(series[0].keys())); w.writeheader(); w.writerows(series)
    if cargo:
        with (root/'cargo.csv').open('w',newline='',encoding='utf-8-sig') as f:
            w=csv.DictWriter(f,fieldnames=list(cargo[0].keys())); w.writeheader(); w.writerows(cargo)
    (root/'summary.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2),encoding='utf-8')
