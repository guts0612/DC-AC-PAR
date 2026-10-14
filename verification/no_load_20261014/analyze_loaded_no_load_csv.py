"""Read the supplied eight-channel VOFA capture; preserve the input CSV.

Phase supplies an estimated time axis because the CSV has no timestamps.
Frequency labels apply to the 2 kframe/s telemetry; high frequencies can alias.
The RMS channel uses full-rate firmware samples, not the telemetry subset.
"""
from pathlib import Path
import hashlib
import json
import numpy as np
try:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib import font_manager
except ModuleNotFoundError:
    plt = None

OUT = Path(__file__).resolve().parent
SOURCE = Path(r"D:\download\vofa\带载和空载对比.csv")
DATA = np.loadtxt(SOURCE, delimiter=",", skiprows=1)
assert DATA.ndim == 2 and DATA.shape[1] == 8 and np.isfinite(DATA).all()
AMPLITUDE = 24.0 * np.sqrt(2.0)
angle = np.arcsin(np.clip(DATA[:, 0] / AMPLITUDE, -1, 1))
ascending = np.gradient(DATA[:, 0]) >= 0
phase = np.unwrap(np.where(ascending, angle, np.pi-angle))
time = (phase-phase[0])/(2*np.pi*50)
increments = np.diff(phase)/(2*np.pi*50/20000)
assert np.all(np.diff(time)>0)
names = ["reference_voltage_v", "sampled_voltage_v", "current_reference_a",
         "sampled_master_current_a", "bridge_reference_v", "sampled_slave_current_a",
         "sampled_bus_v", "full_rate_voltage_rms_v"]
voltage_step = 3.0/2048.0*12.8
raw_float = 2045.0-DATA[:,1]/voltage_step
raw = np.rint(raw_float).astype(int)
assert np.max(np.abs(raw_float-raw))<0.001
assert raw.min()>=0 and raw.max()<=4095
rail = (raw==0)|(raw==4095)
correction = DATA[:,4]-DATA[:,0]
# Boundaries deliberately exclude the switching transient. Zero-based row slices.
segments = {"loaded":(5000,26000), "no_load":(28000,48000)}


def welch(values, fs=2000, nperseg=2048):
    """Average Hann-window spectra with 50 percent overlap using bundled NumPy."""
    window = np.hanning(nperseg)
    pieces = np.lib.stride_tricks.sliding_window_view(values,nperseg)[::nperseg//2]
    pieces = pieces-pieces.mean(axis=1,keepdims=True)
    power = np.abs(np.fft.rfft(pieces*window,axis=1))**2/(fs*np.sum(window**2))
    power[:,1:-1] *= 2
    return np.fft.rfftfreq(nperseg,1/fs),power.mean(axis=0)


def harmonic_fit(values, angles):
    design = np.column_stack([np.ones(len(angles)),np.sin(angles),np.cos(angles)])
    coefficients, *_ = np.linalg.lstsq(design,values,rcond=None)
    residual = values-design@coefficients
    return dict(dc=float(coefficients[0]),
                fundamental_rms=float(np.hypot(*coefficients[1:])/np.sqrt(2)),
                fundamental_phase_deg=float(np.degrees(np.arctan2(coefficients[2],coefficients[1]))),
                residual_rms=float(np.sqrt(np.mean(residual**2)))), residual


results = dict(source=str(SOURCE),sha256=hashlib.sha256(SOURCE.read_bytes()).hexdigest(),
               rows=len(DATA),columns=names,
               duration_estimated_s=float(time[-1]),
               timing_method="50 Hz reference phase; no original timestamps",
               control_sample_increment_quantiles=np.quantile(increments,[0,.01,.5,.99,1]).tolist(),
               apparent_missing_frames=int(np.sum(increments>15)),
               rail_voltage_limits_v=[float(DATA[raw==4095,1][0]),float(DATA[raw==0,1][0])],
               total_rail_frames=int(rail.sum()),
               first_rail_zero_based_row=int(np.flatnonzero(rail)[0]),
               first_rail_estimated_time_s=float(time[np.flatnonzero(rail)[0]]),segments={})
residuals = {}
for key,(start,stop) in segments.items():
    part = DATA[start:stop]
    fitted = {}
    for column in [0,1,2,3,4,5,6]:
        fitted[names[column]],residual = harmonic_fit(part[:,column],phase[start:stop])
        residuals[(key,column)] = residual
    # Count independent RMS windows rather than repeated 40-frame values.
    changes = np.r_[True,np.abs(np.diff(part[:,7]))>1e-7]
    rms_values = part[changes,7]
    frequencies,power = welch(residuals[(key,1)],fs=2000,nperseg=2048)
    mask = frequencies>=200
    top = np.argsort(power[mask])[-5:][::-1]
    band_power = float(np.trapezoid(power[(frequencies>=900)&(frequencies<=1000)],
                                   frequencies[(frequencies>=900)&(frequencies<=1000)]))
    entry=dict(rows_zero_based=[start,stop],
               csv_data_rows_one_based=[start+1,stop],
               estimated_time_s=[float(time[start]),float(time[stop-1])],
               full_rate_rms_frame_weighted=dict(mean=float(part[:,7].mean()),std=float(part[:,7].std()),
                                                min=float(part[:,7].min()),max=float(part[:,7].max()),
                                                p05=float(np.quantile(part[:,7],.05)),
                                                median=float(np.median(part[:,7])),p95=float(np.quantile(part[:,7],.95))),
               full_rate_rms_unique_window=dict(count=len(rms_values),mean=float(rms_values.mean()),std=float(rms_values.std())),
               telemetry_rms=np.sqrt(np.mean(part**2,axis=0)).tolist(),
               harmonic_fit=fitted,
               rail_frames=int(rail[start:stop].sum()),
               rail_fraction=float(rail[start:stop].mean()),
               adc0_frames=int((raw[start:stop]==0).sum()),
               adc4095_frames=int((raw[start:stop]==4095).sum()),
               correction_min=float(correction[start:stop].min()),correction_max=float(correction[start:stop].max()),
               correction_clip_frames=int((np.abs(correction[start:stop])>4.999).sum()),
               bus_mean=float(part[:,6].mean()),bus_std=float(part[:,6].std()),
               bus_rms_correlation=float(np.corrcoef(part[:,6],part[:,7])[0,1]),
               sampled_non50_spectrum_top_hz=frequencies[mask][top].tolist(),
               sampled_900_1000hz_power_v2=band_power,
               frequency_limitation="telemetry-domain frequencies; aliases cannot establish the physical oscillation frequency")
    results["segments"][key]=entry
loaded=results["segments"]["loaded"]
unloaded=results["segments"]["no_load"]
results["sample_domain_load_regulation_percent"]=100*abs(unloaded["full_rate_rms_frame_weighted"]["mean"]-loaded["full_rate_rms_frame_weighted"]["mean"])/unloaded["full_rate_rms_frame_weighted"]["mean"]
results["sampled_fundamental_load_regulation_percent"]=100*abs(unloaded["harmonic_fit"]["sampled_voltage_v"]["fundamental_rms"]-loaded["harmonic_fit"]["sampled_voltage_v"]["fundamental_rms"])/unloaded["harmonic_fit"]["sampled_voltage_v"]["fundamental_rms"]
results["meter_readings_user_reported_v"]={"loaded":[24.0,24.1],"no_load":[24.5,24.7]}
results["physical_si_from_meter_range_percent"]=[100*(24.5-24.1)/24.5,100*(24.7-24.0)/24.7]
results["limitations"]=["Voltage ADC reaches both rails in no-load; sampled RMS cannot recover clipped peaks.",
                         "The current columns are current-reference I2 and actual master-current I3.",
                         "Voltage RMS channel has 20 kHz source samples, whereas waveform telemetry is about 2 kHz.",
                         "Sampled distortion does not uniquely distinguish physical LC oscillation from analog/ADC interference."]
(OUT/"loaded_no_load_csv_analysis.json").write_text(json.dumps(results,ensure_ascii=False,indent=2)+"\n",encoding="utf-8")

if plt is None:
    print("Numerical audit saved. Figures are rendered separately with MATLAB.")
    for key,entry in results["segments"].items():
        print(key, "CH8",entry["full_rate_rms_frame_weighted"],
              "voltage fit",entry["harmonic_fit"]["sampled_voltage_v"],
              "rail frames",entry["rail_frames"],"bus mean",entry["bus_mean"])
    print("sample SI%",results["sample_domain_load_regulation_percent"],
          "50Hz SI%",results["sampled_fundamental_load_regulation_percent"])
    raise SystemExit(0)

for family in ["Microsoft YaHei","SimHei","Noto Sans CJK SC"]:
    if any(font.name==family for font in font_manager.fontManager.ttflist):
        plt.rcParams["font.sans-serif"]=[family]
        break
plt.rcParams["axes.unicode_minus"]=False
plt.rcParams.update({"font.size":11,"axes.grid":True,"grid.alpha":.22})
colors={"loaded":"#2563eb","no_load":"#d45520"}
switch_blocks=np.arange(0,len(DATA)-40,40)
block_current=np.array([np.sqrt(np.mean(DATA[s:s+40,3]**2)) for s in switch_blocks])
cross=np.flatnonzero((block_current[:-1]>1)&(block_current[1:]<1))
transition_row=int(switch_blocks[cross[0]+1]) if len(cross) else 26600
results["load_removal_estimated_time_s"]=float(time[transition_row])
(OUT/"loaded_no_load_csv_analysis.json").write_text(json.dumps(results,ensure_ascii=False,indent=2)+"\n",encoding="utf-8")
fig,axes=plt.subplots(3,1,figsize=(12,8.8),sharex=True,layout="constrained")
axes[0].plot(time,DATA[:,7],color="#d45520",lw=.65,label="CH8：20 kHz 全速采样的一周期 RMS")
axes[0].axhline(24,color="#475569",ls="--",lw=.9,label="24 V 目标")
axes[0].set_ylabel("输出 RMS / V");axes[0].legend(loc="upper left",fontsize=9)
axes[0].set_title("带载转空载：全速 RMS 增大，电压采样开始触及满量程")
axes[1].plot(time[switch_blocks],block_current,color="#2563eb",label="CH4：每 40 个遥测点的电感电流 RMS")
axes[1].set_ylabel("主机电流 / A");axes[1].legend(loc="upper right",fontsize=9)
rail_percent=np.array([100*rail[s:s+400].mean() for s in range(0,len(DATA)-400,400)])
rail_time=np.array([time[s+200] for s in range(0,len(DATA)-400,400)])
axes[2].plot(rail_time,rail_percent,color="#7c3aed",label="CH2：触及 ADC0/4095 的遥测点比例")
axes[2].set_ylabel("满量程点 / %");axes[2].set_xlabel("由 50 Hz 参考反推的时间 / s")
axes[2].legend(loc="upper left",fontsize=9)
for ax in axes:
    ax.axvline(time[transition_row],color="#64748b",ls="--",lw=.9)
    ax.axvspan(time[5000],time[25999],color=colors["loaded"],alpha=.045)
    ax.axvspan(time[28000],time[47999],color=colors["no_load"],alpha=.04)
fig.savefig(OUT/"loaded_no_load_capture_overview.png",dpi=160)
plt.close(fig)

fig,axes=plt.subplots(2,2,figsize=(13,7.5),layout="constrained")
for key,base in [("loaded",10000),("no_load",32000)]:
    column=0 if key=="loaded" else 1
    sl=slice(base,base+120)
    local_ms=(time[sl]-time[base])*1000
    ax=axes[0,column]
    ax.plot(local_ms,DATA[sl,0],color="#64748b",ls="--",label="参考")
    ax.plot(local_ms,DATA[sl,1],color=colors[key],lw=1,label="输出电压采样")
    local_rail=rail[sl]
    ax.scatter(local_ms[local_rail],DATA[sl,1][local_rail],marker="x",color="#b91c1c",s=36,label="ADC 满量程")
    ax.set_title("12 Ω 带载" if key=="loaded" else "空载")
    ax.set_ylabel("电压 / V");ax.set_xlabel("时间 / ms");ax.legend(fontsize=9)
    axes[1,column].plot(local_ms,DATA[sl,3],color=colors[key],lw=1,label="CH4：实测电感电流")
    axes[1,column].plot(local_ms,DATA[sl,2],color="#64748b",lw=.8,ls="--",label="CH3：电流参考")
    axes[1,column].set_ylabel("电流 / A");axes[1,column].set_xlabel("时间 / ms");axes[1,column].legend(fontsize=9)
fig.suptitle("原始遥测波形：空载畸变与采样削顶（波形通道约 2 kHz，无法识别真实高频）")
fig.savefig(OUT/"loaded_no_load_waveforms.png",dpi=160)
plt.close(fig)
print("rows",len(DATA),"duration estimated",time[-1],"switch estimated",time[transition_row])
for key,entry in results["segments"].items():
    print(key, "CH8",entry["full_rate_rms_frame_weighted"],"fitted voltage",entry["harmonic_fit"]["sampled_voltage_v"],
          "true current",entry["harmonic_fit"]["sampled_master_current_a"],"rail",entry["rail_frames"],entry["rail_fraction"],
          "bus",entry["bus_mean"],"correction",entry["correction_min"],entry["correction_max"])
print("sample SI%",results["sample_domain_load_regulation_percent"],"50Hz SI%",results["sampled_fundamental_load_regulation_percent"])
