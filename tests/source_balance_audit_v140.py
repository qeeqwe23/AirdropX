from pathlib import Path
import sys
ROOT=Path(__file__).resolve().parents[1]
files=[
 ROOT/'matlab/airdrop/airdropx_wind_airdrop_mission_v140.m',
 ROOT/'matlab/airdrop/airdropx_wind_airdrop_entry_v140.m',
 ROOT/'matlab/airdrop/airdropx_phys_mpc_base_equivalence_audit_v140.m',
 ROOT/'matlab/airdrop/airdropx_wind_airdrop_finalize_v140.m',
 ROOT/'matlab/airdrop/airdropx_cargo_truth_params_v140.m',
 ROOT/'matlab/airdrop/airdropx_cargo_truth_plant_v140.m',
 ROOT/'matlab/avionics/airdropx_avionics_init_v140.m',
 ROOT/'matlab/avionics/airdropx_avionics_step_v140.m',
 ROOT/'matlab/phys_mpc/airdropx_phys_mpc_wind_disturbance_fit_flightlog_v140.m',
 ROOT/'run_wind_disturbance_airdrop_v140_D.ps1',
 ROOT/'run_wind_disturbance_airdrop_point_v140_D.ps1',
 ROOT/'run_base_equivalence_audit_v140_D.ps1',
 ROOT/'install_wind_disturbance_airdrop_v140.ps1',
]

def balance(text, pairs, single_quotes=False):
    counts={o:0 for o in pairs}
    stack=[]; quote=None; i=0; line=1
    inv={v:k for k,v in pairs.items()}
    while i<len(text):
        c=text[i]
        if c=='\n': line+=1
        if quote:
            if c==quote:
                # escaped doubled quotes
                if i+1<len(text) and text[i+1]==quote:
                    i+=2; continue
                quote=None
            i+=1; continue
        if c=='"' or (single_quotes and c=="'"):
            quote=c; i+=1; continue
        # strip % comments for matlab and # comments for ps only when at line content; simplified
        if c=='%' and text[max(0,text.rfind('\n',0,i)+1):i].find("'")<0:
            j=text.find('\n',i); i=len(text) if j<0 else j; continue
        if c=='#':
            j=text.find('\n',i); i=len(text) if j<0 else j; continue
        if c in pairs:
            stack.append((c,line))
        elif c in inv:
            if not stack or stack[-1][0]!=inv[c]: return False,f'unmatched {c} line {line}'
            stack.pop()
        i+=1
    return (not stack, 'ok' if not stack else f'unclosed {stack[-1][0]} line {stack[-1][1]}')

okall=True
for f in files:
    text=f.read_text()
    pairs={'(':')','[':']','{':'}'} if f.suffix.lower()=='.ps1' else {'(':')','[':']','{':'}'}
    ok,msg=balance(text,pairs,single_quotes=(f.suffix.lower()=='.ps1'))
    print(f"{'PASS' if ok else 'FAIL'} {f.name}: {msg}")
    okall &= ok
if not okall: sys.exit(1)
