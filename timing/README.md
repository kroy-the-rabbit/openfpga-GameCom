# Game.com tech demo timing

**PASS** | Quartus Prime Lite 25.1std.0 Build 1129 | 5CEBA4F23C8 | Standard Fit | Seed 1

| Corner | Setup (ns) | Hold (ns) | Recovery (ns) | Removal (ns) | Minimum pulse width (ns) |
|---|---:|---:|---:|---:|---:|
| Slow 1100mV 85C | +0.603 | +0.284 | +11.183 | +0.809 | +0.830 |
| Slow 1100mV 0C | +1.001 | +0.279 | +11.309 | +0.755 | +0.830 |
| Fast 1100mV 85C | +3.660 | +0.132 | +12.334 | +0.285 | +0.830 |
| Fast 1100mV 0C | +3.778 | +0.084 | +12.403 | +0.258 | +0.830 |

| Resource | Used | Available |
|---|---:|---:|
| ALMs | 12,430 | 18,480 |
| Memory bits | 419,136 | 3,153,920 |
| RAM blocks | 62 | 308 |
| DSP blocks | 0 | 66 |
| PLLs | 2 | 4 |

Total negative slack: **0 ns**. Unconstrained clocks and I/O: **0**. CDC checks: **96 / 96 pass**.

[Fitter summary](quartus-reports/ap_core.fit.summary) | [Fitter report](quartus-reports/ap_core.fit.rpt) | [Timing summary](quartus-reports/ap_core.sta.summary) | [Timing report](quartus-reports/ap_core.sta.rpt)

[Clocks and Fmax](clock-reports/) | [CDC slack](cdc-slack.tsv) | [Data and hashes](summary.json) | [Checksums](SHA256SUMS)

Bitstream SHA256:

```text
8f39fcca163ead4d3709553938e90296b2ddfdb93dba1b9d1128c9cd69ac8279
```
