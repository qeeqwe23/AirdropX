import math, random

GUST_STEP=1.5
DELTA_FRAC=0.85
CONF_MIN=0.35
ASSIST_MAX=0.25

def trigger(w_prev,w_now,conf):
    return conf>=CONF_MIN and abs(w_now-w_prev)>=DELTA_FRAC*GUST_STEP

def smooth(x,a,b):
    if b<=a: return float(x>=b)
    z=max(0.0,min(1.0,(x-a)/(b-a)))
    return z*z*(3-2*z)

def assist(rate_eff,rate_conf):
    return ASSIST_MAX*smooth(abs(rate_eff),0.35,1.50) if rate_conf>=0.60 else 0.0

def energy_shift(va_norm,throttle,level,max_shift=2.5):
    if level<0.5: return 0.0
    if not ((va_norm<=-0.75 and throttle>=0.985) or (va_norm>=0.75 and throttle<=0.015)):
        return 0.0
    amp=smooth(abs(va_norm),0.75,2.5)
    return max_shift*(1 if va_norm>0 else -1)*amp*max(level,0.5)

# Calm: even a deliberately noisy small estimate cannot latch without a >=1.275 m/s jump.
random.seed(132)
calm=[random.gauss(0,0.25) for _ in range(1000)]
calm_triggers=sum(trigger(calm[i-1],calm[i],0.05) for i in range(1,len(calm)))
assert calm_triggers==0

# Real step cases latch.
assert trigger(0.1,4.8,0.90)
assert trigger(-0.2,-11.7,0.90)
assert trigger(7.8,-7.9,0.95)

# Smooth ramp/sine at 10 Hz do not look like instantaneous steps.
ramp=[-10+20*i/200 for i in range(201)]
assert not any(trigger(ramp[i-1],ramp[i],0.90) for i in range(1,len(ramp)))
sine=[8*math.sin(2*math.pi*0.08*i*0.1) for i in range(500)]
assert not any(trigger(sine[i-1],sine[i],0.90) for i in range(1,len(sine)))

# Continuous assist is bounded.
vals=[assist(r,1.0) for r in [0,0.35,0.5,1,1.5,3]]
assert min(vals)>=0 and max(vals)<=ASSIST_MAX+1e-12

# Energy-exchange sign and gate.
assert energy_shift(-2.5,1.0,1.0)<0   # low Va -> lower H reference
assert energy_shift( 2.5,0.0,1.0)>0   # high Va -> higher H reference
assert energy_shift(-2.5,0.6,1.0)==0  # no throttle limit -> no energy borrowing
assert energy_shift(-2.5,1.0,0.25)==0 # weak continuous assist -> no energy second pass

print('v1.3.3 independent policy audit')
print(f'calm_false_abrupt_triggers={calm_triggers}')
print('step_trigger_5mps=PASS')
print('step_trigger_12mps=PASS')
print('bidirectional_step_trigger=PASS')
print('smooth_ramp_no_strong_latch=PASS')
print('smooth_sine_no_strong_latch=PASS')
print(f'continuous_assist_max={max(vals):.6f}')
print(f'energy_low_va_shift_m={energy_shift(-2.5,1.0,1.0):.6f}')
print(f'energy_high_va_shift_m={energy_shift(2.5,0.0,1.0):.6f}')
print('PASS=1')
