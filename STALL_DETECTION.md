# Motor Telemetry Fields

The following fields expose real-time diagnostics from the TMC driver and the motor control layer. They are primarily used to characterize load, tune current, and configure stall detection.

---

**`sg` — StallGuard Result (`SG_RESULT`, range 0–510)**  
Represents the instantaneous load measurement derived from the motor’s back-EMF.  

- During normal motion, values are typically in the mid-to-high range (e.g., ~150–400, depending on velocity and tuning).  
- As mechanical load increases, this value drops toward 0.  
- This is the primary signal used when calibrating stall detection thresholds.

---

**`cs` — Actual Current Scale (`CS_ACTUAL`, range 0–31)**  
Indicates the effective current level being driven by the controller.  

- Under nominal conditions, this remains relatively stable and reflects the configured run current.  
- Under increased load, the driver raises this value to maintain torque output.

---

**`stl` — Standstill Flag**  
Indicates whether the driver considers the motor to be at standstill.  

- `0`: Motor is in motion (steps are being received).  
- Even under load, this remains `0` as long as motion commands are active.

---

**`sth` — StealthChop Active Flag**  
Indicates whether StealthChop mode is active.  

- `0`: SpreadCycle mode  
- `1`: StealthChop mode  
- This value is independent of load and reflects the configured drive mode.

---

**`sg_score` — Stall Detection Score (internal)**  
Internal metric used by the driver’s stall detection logic.  

- Typically negative during normal operation (no stall condition).  
- Increases toward a limit as load approaches stall conditions.

---

**`baseline` — Expected SG Value**  

Reference StallGuard value for the current speed.  

- Only populated when stall detection has been explicitly configured.  
- Used internally to normalize stall detection behavior across speeds.

---

**`spd` — Step Rate (steps/sec)**  
Commanded step frequency from the motor control layer (`lib_motor`).  

- Reflects the current motion profile (jogging, cycling, etc.).  
- Remains constant regardless of load unless motion control changes it.

---

**`mov` — Motion Flag**  
Indicates whether the motor is actively moving.

- `1`: Motor is in motion  
- Remains `1` under load as long as steps are being issued.

---

## Stall Detection Notes

Fields related to stall detection (`sg_score`, `baseline`, and any threshold/diagnostic signals such as `diag_sc` or `thr`) are only meaningful after stall detection has been configured via:

```cpp
tmc_.configureStall(sensitivity != 0);
```

Until this is done, these fields will report zero or inactive values.

For initial tuning and load characterization, sg and cs are sufficient. In particular, sg should be profiled across the expected operating range and used to derive appropriate StallGuard thresholds.
