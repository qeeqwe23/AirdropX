# v1.3.2 implementation rationale from the completed v1.3.1 run

Observed v1.3.1 aggregate facts used to choose the changes:

- 24/24 three-way missions executed; infrastructure was no longer the limiter.
- Wind-aware worst landing max: 3.9237 m; no-wind release worst max: 78.9628 m.
- Calm fractional release RMS: 2.2887 m; sampled release RMS: 3.4011 m. Fractional timing therefore stays.
- Calm wind-aware RMS (2.2887 m) was worse than calm no-wind RMS (1.7741 m), motivating significance gating and release-state noise filtering rather than a calm-only offset.
- Sine carrier RMS improved from 1.4897 to 1.2076 and ramp from 1.1033 to 0.9907, so physical `Gw` preview stays.
- Worst recovery remained 13.2 s; headwind 12 recovery was 11.1 s vs legacy 11.5 s. This motivates a post-gust recovery policy instead of simply increasing `Gw`.
- No formal gate is loosened in v1.3.2.
