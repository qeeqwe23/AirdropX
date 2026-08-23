function T=airdropx_phys_interval_case_manifest_v090(outputPath)
%AIRDROPX_PHYS_INTERVAL_CASE_MANIFEST_V090 Two deterministic off-grid samples in every interpolation cell.
h0=20:10:190; v0=[45 50 55 60]; n=numel(h0)*numel(v0)*2;
CaseId=strings(n,1); H_m=zeros(n,1); V_mps=zeros(n,1); Pattern=strings(n,1); HCellLow=zeros(n,1); VCellLow=zeros(n,1); k=0;
for iv=1:numel(v0)
    for ih=1:numel(h0)
        k=k+1; H=h0(ih)+5; V=v0(iv)+2.5; CaseId(k)=localId("C",H,V); H_m(k)=H; V_mps(k)=V; Pattern(k)="center"; HCellLow(k)=h0(ih); VCellLow(k)=v0(iv);
        k=k+1;
        if mod(ih+iv,2)==0, H=h0(ih)+2.5; V=v0(iv)+3.75; else, H=h0(ih)+7.5; V=v0(iv)+1.25; end
        CaseId(k)=localId("Q",H,V); H_m(k)=H; V_mps(k)=V; Pattern(k)="staggered_quarter"; HCellLow(k)=h0(ih); VCellLow(k)=v0(iv);
    end
end
T=table(CaseId,H_m,V_mps,Pattern,HCellLow,VCellLow);
if nargin>0 && strlength(string(outputPath))>0, writetable(T,string(outputPath)); end
end
function s=localId(prefix,H,V)
s=string(sprintf("%s_H%05d_V%05d",prefix,round(100*H),round(100*V)));
end
