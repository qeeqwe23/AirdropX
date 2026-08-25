from pathlib import Path
import re, ast, sys
ROOT=Path(__file__).resolve().parents[1]
files=[ROOT/'matlab/airdrop/airdropx_wind_airdrop_mission_v140.m', ROOT/'matlab/airdrop/airdropx_wind_airdrop_finalize_v140.m', ROOT/'matlab/phys_mpc/airdropx_phys_mpc_wind_disturbance_fit_flightlog_v140.m']

def find_balanced(text,start):
    # start points at t in table(; return content inside parens
    p=text.index('(', start); depth=1; i=p+1; quote=None
    while i<len(text):
        c=text[i]
        if quote:
            if c==quote:
                # doubled quote inside MATLAB strings
                if i+1<len(text) and text[i+1]==quote:
                    i+=2; continue
                quote=None
            i+=1; continue
        if c in '"': quote=c; i+=1; continue
        # single quote is often transpose; don't treat it as string globally
        if c=='(' : depth+=1
        elif c==')':
            depth-=1
            if depth==0: return text[p+1:i]
        i+=1
    raise ValueError('unbalanced')

def split_top(s):
    out=[]; cur=[]; dp=db=dc=0; quote=None; i=0
    while i<len(s):
        c=s[i]
        if quote:
            cur.append(c)
            if c==quote:
                if i+1<len(s) and s[i+1]==quote:
                    cur.append(s[i+1]); i+=2; continue
                quote=None
            i+=1; continue
        if c=='"': quote='"'; cur.append(c); i+=1; continue
        # Recognize single quoted strings only when preceded by punctuation/space and followed by alpha.
        if c=="'" and i+1<len(s) and (s[i+1].isalpha() or s[i+1]=='{'):
            quote="'"; cur.append(c); i+=1; continue
        if c=='(': dp+=1
        elif c==')': dp-=1
        elif c=='[': db+=1
        elif c==']': db-=1
        elif c=='{': dc+=1
        elif c=='}': dc-=1
        if c==',' and dp==0 and db==0 and dc==0:
            out.append(''.join(cur).strip()); cur=[]
        else: cur.append(c)
        i+=1
    if cur: out.append(''.join(cur).strip())
    return out

def varnames_count(expr):
    m=re.search(r"['\"]VariableNames['\"]\s*,\s*\{(.*)\}\s*$",expr,re.S)
    if not m: return None
    body=m.group(1)
    # all single-quoted names inside the final cell array
    return len(re.findall(r"'([^']+)'",body))

checks=[]
for f in files:
    text=f.read_text().replace('...','')
    for m in re.finditer(r'\btable\s*\(',text):
        # ignore empty table()
        content=find_balanced(text,m.start())
        if 'VariableNames' not in content: continue
        # split at top-level occurrence of VariableNames by looking for the token
        vm=content.find("'VariableNames'")
        if vm<0: vm=content.find('"VariableNames"')
        values=content[:vm].rstrip().rstrip(',')
        vals=split_top(values)
        nvars=varnames_count(content)
        ok=nvars==len(vals)
        line=text[:m.start()].count('\n')+1
        checks.append((f.name,line,len(vals),nvars,ok))
for name,line,nv,nn,ok in checks:
    print(f"{'PASS' if ok else 'FAIL'} {name}:{line} values={nv} names={nn}")
print(f"\ntable_constructor_static_audit_v140: {sum(x[-1] for x in checks)}/{len(checks)} PASS")
if not checks or not all(x[-1] for x in checks): sys.exit(1)
