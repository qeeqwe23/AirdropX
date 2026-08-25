function r=airdropx_wind_synthetic_selftest_v111(outputPath)
%AIRDROPX_WIND_SYNTHETIC_SELFTEST_V111 Independent geometry/noise/step preflight.
Ts=0.1; maxGeom=0;
for Va=[45 50 55 60 65]
    for Vz=[-3 -1 0 1 3]
        Vah=sqrt(Va^2-Vz^2);
        for w=-15:5:15
            Vg=Vah+w;
            m=airdropx_longitudinal_wind_measurement_v111(Va,Vz,Vg);
            maxGeom=max(maxGeom,abs(m.raw_wind_mps-w));
        end
    end
end
t=(0:Ts:35).'; w=zeros(size(t)); w(t>=5)=8; w(t>=15)=-8; w(t>=25)=3;
nseed=100; rmsv=zeros(nseed,1); p95=zeros(nseed,1); settle=zeros(nseed,1);
for seed=1:nseed
    rng(seed,"twister"); Va=50+0.25*randn(size(t)); Vz=0.10*randn(size(t)); Vg=50+w+0.15*randn(size(t));
    s=airdropx_longitudinal_wind_estimator_init_v111(Ts); e=zeros(size(t));
    for k=1:numel(t), [s,o]=airdropx_longitudinal_wind_estimator_step_v111(s,Va(k),Vz(k),Vg(k)); e(k)=o.wind_est_mps; end
    mask=localMask(t,w); er=e-w; rmsv(seed)=sqrt(mean(er(mask).^2)); p95(seed)=localP(abs(er(mask)),95); settle(seed)=localSettle(t,w,e);
end
r=struct("version","Physics-MPC v1.1.3 wind synthetic self-test","geometry_max_error_mps",maxGeom,"mc_rmse_mean_mps",mean(rmsv),"mc_rmse_worst_mps",max(rmsv),"mc_p95_mean_mps",mean(p95),"step_settling90_worst_s",max(settle),"pass",false);
r.pass=maxGeom<=1e-12 && mean(rmsv)<=0.35 && mean(p95)<=0.75 && max(settle)<=0.5;
if nargin>0 && strlength(string(outputPath))>0
    fid=fopen(outputPath,"w"); if fid>=0, c=onCleanup(@()fclose(fid)); fprintf(fid,"Physics-MPC v1.1.3 wind synthetic self-test\n"); fn=fieldnames(r); for i=1:numel(fn), v=r.(fn{i}); if isnumeric(v)&&isscalar(v), fprintf(fid,"%s=%.12g\n",fn{i},v); elseif islogical(v), fprintf(fid,"%s=%d\n",fn{i},v); end, end, end
end
if ~r.pass, error("AirdropX:Wind:SyntheticSelftestFailed","Wind estimator synthetic self-test failed."); end
end
function p=localP(x,q), x=sort(x(:)); pos=1+(numel(x)-1)*q/100; a=floor(pos); b=ceil(pos); if a==b,p=x(a);else,p=x(a)+(pos-a)*(x(b)-x(a));end, end
function mask=localMask(t,w)
mask=t>=2; idx=find(abs([0;diff(w)])>=2);
for j=1:numel(idx), mask=mask & ~(t>=t(idx(j)) & t<=t(idx(j))+.5); end
end
function s=localSettle(t,w,e)
idx=find(abs([0;diff(w)])>=2); vals=zeros(numel(idx),1); holdN=3;
for j=1:numel(idx)
    k=idx(j); amp=abs(w(k)-w(k-1)); tol=.1*amp; ke=numel(t); if j<numel(idx),ke=idx(j+1)-1;end
    vals(j)=Inf;
    for q=k:max(k,ke-holdN+1)
        q2=min(ke,q+holdN-1);
        if q2-q+1==holdN && all(abs(e(q:q2)-w(k))<=tol), vals(j)=t(q)-t(k); break; end
    end
end
s=max(vals);
end
