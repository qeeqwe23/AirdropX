from __future__ import annotations
import argparse
from pathlib import Path
import math
import pandas as pd

TS=0.1
RATE_THRESHOLD=0.70
RATE_CONF=0.70
RATE_ACT_WIND_CONF=0.15
RATE_HOLD_S=0.30
PERSIST_S=3.50

def replay(csv_path: Path):
    d=pd.read_csv(csv_path)
    active=False
    rate_act_count=0
    rate_keep_count=0
    hold_count=0
    events=0
    out=[]
    rate_hold_n=max(1,math.ceil(RATE_HOLD_S/TS))
    persist_n=max(1,math.ceil(PERSIST_S/TS))
    for _,r in d.iterrows():
        gust=bool(r['abrupt_gust_trigger'])
        rate_dynamic=(float(r['wind_rate_confidence'])>=RATE_CONF and abs(float(r['wind_rate_effective_mps2']))>=RATE_THRESHOLD)
        rate_activation=rate_dynamic and float(r['wind_confidence'])>=RATE_ACT_WIND_CONF
        rate_act_count=rate_act_count+1 if rate_activation else 0
        rate_keep_count=rate_keep_count+1 if rate_dynamic else 0
        was=active
        if gust or rate_act_count>=rate_hold_n:
            active=True; hold_count=persist_n
        elif active:
            if bool(r['gust_recovery_latched']) or rate_keep_count>=rate_hold_n:
                hold_count=persist_n
            else:
                hold_count=max(0,hold_count-1)
                if hold_count==0:
                    active=False
        if active and not was:
            events += 1
        out.append(active)
    switches=sum(a!=b for a,b in zip(out[:-1],out[1:]))
    active_idx=[i for i,v in enumerate(out) if v]
    last=float(d.iloc[active_idx[-1]]['t_s']) if active_idx else float('nan')
    return dict(active_fraction=sum(out)/len(out), events=events, switches=switches,last_active_s=last)

def main():
    ap=argparse.ArgumentParser()
    ap.add_argument('root', nargs='?', default='')
    ns=ap.parse_args()
    if ns.root:
        root=Path(ns.root)
    else:
        # Useful when this script is run from the delivered package beside an
        # extracted v1.3.5 three-point result folder.
        root=Path('physics_mpc_v135_equivalent_energy_recovery_point')
    scenarios=['calm_wind_mpc_aware','headwind_12_wind_mpc_aware','sine_longitudinal_wind_mpc_aware']
    for sc in scenarios:
        p=root/sc/'wind_airdrop_timeseries.csv'
        if not p.is_file():
            print(f'{sc}: SKIP missing {p}')
            continue
        r=replay(p)
        print(f"{sc}: transient_fraction={r['active_fraction']:.9f} events={r['events']} switches={r['switches']} last_active_s={r['last_active_s']:.3f}")

if __name__=='__main__':
    main()
