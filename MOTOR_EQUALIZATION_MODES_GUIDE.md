# Motor Speed Equalization Modes - Quick Reference Guide

## 🎯 Which Mode Should I Use?

### START HERE: Do you have rotation during stops or direction changes?
**YES** → Use **ACCELERATION_SYNC** mode ✅ (Most common issue)  
**NO** → Try AGGRESSIVE or CONSERVATIVE for steady-state speed issues

---

## Mode Comparison

### 🚀 ACCELERATION_SYNC (NEW - RECOMMENDED)
**What it fixes:** Rotation during deceleration, unpredictable behavior during direction changes

**How it works:**
```
Problem: During a quick stop or direction change
❌ Left motors decelerate fast  → Robot rotates right
❌ Right motors decelerate slow → 

Solution: ACCELERATION_SYNC detects this and:
✅ Adds power to left motors  → All motors decelerate
✅ Keeps right motors as-is  → together smoothly
```

**Use when:**
- ⚠️ Robot rotates when stopping
- ⚠️ Robot behaves unpredictably during direction changes
- ⚠️ Some wheels stop before others
- You do lots of quick maneuvers

**Configuration:**
```java
drive.setSpeedEqualizationMode(CorrectionMode.ACCELERATION_SYNC);
equalizer.setAccelerationCorrectionFactor(0.20);  // 20% max correction
```

---

### ⚡ AGGRESSIVE Mode
**What it fixes:** One side consistently slower during steady movement

**How it works:**
```
Problem: During forward movement
❌ Left motors: 80% speed  → Robot drifts right
❌ Right motors: 100% speed →

Solution: AGGRESSIVE mode:
✅ Boosts left motors +15% → All motors at similar speed
✅ Right motors unchanged →
```

**Use when:**
- Robot drifts to one side during straight movement
- You want maximum speed
- Motors have power headroom (not at max)

**Limitation:** Won't work if slow motors are already at max power

**Configuration:**
```java
drive.setSpeedEqualizationMode(CorrectionMode.AGGRESSIVE);
equalizer.setAggressiveBoostFactor(0.15);  // 15% max boost
```

---

### 🎯 CONSERVATIVE Mode
**What it fixes:** One side consistently faster, need to guarantee sync

**How it works:**
```
Problem: During forward movement
❌ Left motors: 100% speed → Robot drifts left
❌ Right motors: 80% speed →

Solution: CONSERVATIVE mode:
✅ Left motors reduced to 80% → All motors synchronized
✅ Right motors unchanged →
```

**Use when:**
- Precision is more important than speed
- Battery life is a concern
- You need guaranteed synchronization

**Limitation:** Robot moves slower overall

**Configuration:**
```java
drive.setSpeedEqualizationMode(CorrectionMode.CONSERVATIVE);
equalizer.setConservativeReductionFactor(0.15);  // 15% max reduction
```

---

### ❌ DISABLED Mode
Turns off all equalization. Use for:
- Testing baseline robot behavior
- When equalization causes issues
- Well-balanced robots that don't need correction

---

## 📊 Decision Flow Chart

```
Do you have problems?
│
├─ During STOPPING/DIRECTION CHANGES?
│  └─ Robot rotates or unpredictable behavior
│     └─ ✅ Use ACCELERATION_SYNC
│
├─ During STEADY MOVEMENT?
│  ├─ One side slower, want max speed
│  │  └─ ✅ Use AGGRESSIVE
│  │
│  └─ One side faster, want precision
│     └─ ✅ Use CONSERVATIVE
│
└─ No problems or need to test
   └─ Use DISABLED
```

---

## 🧪 Testing Procedure

1. **Start with DISABLED**
   - Drive the robot around
   - Note what problems occur (rotation during stops, drift during movement, etc.)

2. **If rotation during stops/direction changes:**
   ```java
   drive.setSpeedEqualizationMode(CorrectionMode.ACCELERATION_SYNC);
   ```
   - Test quick direction changes
   - Test sudden stops
   - Should see less rotation

3. **If drift during steady movement:**
   - Try AGGRESSIVE first (for speed)
   - If that doesn't work (motors at max), try CONSERVATIVE

4. **Fine-tune parameters:**
   - If corrections too weak: increase correction factor
   - If corrections too strong: decrease correction factor
   - Monitor telemetry to see what's happening

---

## 📈 Telemetry Interpretation

```java
Motor Data:
  LF: 1250 t/s, -450 t/s² [+0.12]   ← Speed, Acceleration, Correction
  RF: 1300 t/s, -350 t/s² [+0.00]
  LB: 1200 t/s, -480 t/s² [+0.15]
  RB: 1280 t/s, -340 t/s² [+0.00]
```

**What to look for:**

**ACCELERATION_SYNC mode:**
- Negative accelerations = deceleration
- Different deceleration rates (e.g., LF at -450, RF at -350) = problem
- Corrections applied to synchronize them

**AGGRESSIVE/CONSERVATIVE modes:**
- Different speeds (e.g., LF at 1200, RF at 1300) = problem  
- Corrections applied to match speeds

---

## 💡 Pro Tips

1. **Start with ACCELERATION_SYNC** - It fixes the most common issue (deceleration imbalance)

2. **Monitor telemetry** - Watch the acceleration values during direction changes

3. **Adjust correction factors gradually:**
   - Start at 0.10 (10%)
   - Increase by 0.05 until problems are fixed
   - Don't go above 0.30 (30%) - may cause oscillation

4. **Test in realistic scenarios:**
   - With game pieces loaded
   - With lift/manipulator extended
   - During actual match movements

5. **Combine with SmartMechanumDrive features:**
   - Acceleration limiting
   - Heading stabilization
   - Drive modes (Sport, Precision, etc.)

---

## 🔧 Common Issues & Solutions

### "ACCELERATION_SYNC makes it worse"
- **Solution:** Reduce `accelerationCorrectionFactor` from 0.20 to 0.10
- Check that motors are in RUN_USING_ENCODER mode

### "Still rotates during stops"
- **Solution:** Increase `accelerationCorrectionFactor` to 0.25 or 0.30
- Verify uneven weight distribution is the cause (not mechanical binding)

### "Robot moves slower than before"
- **Solution:** Switch from CONSERVATIVE to ACCELERATION_SYNC or AGGRESSIVE
- CONSERVATIVE intentionally slows to match slowest motor

### "Corrections not being applied"
- Check `drive.setSpeedEqualizationEnabled(true)` is called
- Verify mode is not DISABLED
- Check telemetry - may be below speed/acceleration thresholds

---

## 📝 Summary

**Most robots should start with ACCELERATION_SYNC mode** - it directly addresses the most common issue of uneven deceleration during direction changes.

Only use AGGRESSIVE/CONSERVATIVE if you have steady-state speed differences that ACCELERATION_SYNC doesn't fix.
