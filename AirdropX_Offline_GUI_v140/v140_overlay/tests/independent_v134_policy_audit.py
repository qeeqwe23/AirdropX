# Small independent policy sanity check; no MATLAB model claims are made.
def energy_cap(h, abs_cap=3.0, frac=0.02): return min(abs_cap, frac*max(abs(h),1))
assert abs(energy_cap(200)-3.0)<1e-12
assert abs(energy_cap(100)-2.0)<1e-12
assert abs(energy_cap(20)-0.4)<1e-12
# With continuous recovery assist disabled and no abrupt gust, recovery target is zero.
def recovery_target(gust_latched,state_severity,assist_enabled=False,assist=0.25):
    a=assist if assist_enabled else 0.0
    return max(state_severity,a) if gust_latched else a
assert recovery_target(False,1.0,False)==0.0
assert recovery_target(False,1.0,True)==0.25
# Constant Iyy origin offsets cancel under a cfg0-bias-corrected delta comparison.
expected=[1000.0,900.0,800.0]
bias=0.6119
observed=[x+bias for x in expected]
ref=[x+(observed[0]-expected[0]) for x in expected]
assert max(abs(a-b) for a,b in zip(observed,ref))<1e-12
print('PASS v1.3.4 independent policy audit')
print('energy caps: H200=3.0 m, H100=2.0 m, H20=0.4 m')
print('no abrupt gust + assist disabled -> recovery target 0')
print('constant Iyy origin bias cancels without relaxing cfg delta check')
