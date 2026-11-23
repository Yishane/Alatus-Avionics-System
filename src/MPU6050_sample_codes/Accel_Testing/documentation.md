# Accelerometer Testing: Finding the Plane's Angle with an MPU-6050

This program reads the accelerometer on an MPU-6050, uses gravity to figure out how far the sensor is tilted, and prints the roll and pitch angles to the Serial Monitor.

The output looks like this:

```
Roll:  0.12   |  Pitch:  -0.05
Roll:  15.40   |  Pitch:  -0.31
Roll:  29.87   |  Pitch:  2.14
```

All angles are in degrees.

---

## 1. The big idea

The gyro program worked by adding up how fast the sensor was spinning. This one works in a completely different way:

**An accelerometer can feel gravity, and gravity always points down. If we know which way "down" is from the sensor's point of view, we know how tilted it is.**

Think about holding a glass of water. If you tilt the glass, the water stays level, and by looking at the water you can tell how far you've tilted it. The accelerometer is basically doing that, except instead of water it's measuring how much of gravity's pull lands on each of its 3 axes.

- Sitting flat on a desk, all of gravity (1 g) is on the **Z** axis. X and Y read about 0.
- Tilt it 90° onto its side, and now all of gravity is on **X** or **Y** instead, and Z reads about 0.
- Anywhere in between, gravity gets split up between the axes.

Using some trig (section 9), we turn that split back into an angle.

The nice thing about this compared to the gyro: **there's no adding up over time, so there's no drift.** Every reading gives a fresh angle on its own. The downside is in section 12.

(**g** is the unit we use for acceleration here. 1 g is the pull of gravity at Earth's surface, about 9.81 m/s².)

---

## 2. Roll and pitch (but no yaw)

With the board sitting flat and the X arrow pointing at the nose of the plane:

| Axis | Name | What it looks like on a plane |
|---|---|---|
| X | **Roll** | One wing goes up, the other goes down |
| Y | **Pitch** | Nose goes up or down |

There's no **yaw** in this program, and that's not a mistake. Yaw is turning left or right, like shaking your head "no". When you do that, gravity still points straight down through the sensor the same way it did before, so the accelerometer can't tell anything changed. Yaw needs a gyro or a compass.

If the board is mounted a different way on the plane, these names get swapped around, so check which way the arrows on the board point.

---

## 3. Wiring

Same as the gyro, since it's the same chip.

| MPU-6050 | Arduino Nano |
|---|---|
| VCC | 5V |
| GND | GND |
| SCL | A5 |
| SDA | A4 |

SCL and SDA are the two wires for **I2C**, which is how the Arduino talks to the sensor. The gyro documentation ([../Gyro_testing/documentation.md](../Gyro_testing/documentation.md)) goes through I2C and the `Wire.h` functions in more detail.

---

## 4. What the program does, in order

**Once, when it turns on (`setup()`):**

1. Start the Serial Monitor connection and the I2C bus.
2. **Wake up the MPU-6050.** It starts in sleep mode, so we have to tell it to get up.
3. **Set the accelerometer range** to ±8 g.
4. **Calibrate** (`calculate_IMU_error()`). The sensor has to sit **flat and perfectly still** for about 200 ms here.

**Indefinitely after that (`loop()`):**

1. Wait until 1 millisecond has passed.
2. Read the accelerometer (`readAccelerometer()`).
3. Subtract the calibration error.
4. Use trig to turn the X, Y, Z readings into roll and pitch (section 9).
5. Every 20 ms, print the angles.

---

## 5. The registers we use

The MPU-6050 stores its settings and readings in numbered slots called **registers** (the gyro documentation compares them to school lockers). The ones this program uses:

| Register | What it's for | What we do |
|---|---|---|
| `0x6B` | Power management | Write `0x00` to wake it up and use the internal 8 MHz clock |
| `0x1C` | Accelerometer settings | Write `0x10` to set the range to ±8 g |
| `0x3B` to `0x40` | The accelerometer readings | Read these every millisecond |

(The `0x` just means the number is written in hexadecimal. `0x1C` is 16 + 12 = 28 in normal numbers.)

### Why `0x10` means ±8 g

The range is picked using 2 bits inside register `0x1C` (bits 4 and 3, called `AFS_SEL` in the datasheet):

| `AFS_SEL` | Byte we write | Range |
|---|---|---|
| 0 | `0x00` | ±2 g |
| 1 | `0x08` | ±4 g |
| **2** | **`0x10`** | **±8 g (what we use)** |
| 3 | `0x18` | ±16 g |

`0x10` in binary is `00010000`. Bits 4 and 3 are `10`, which is 2, so that's ±8 g.

---

## 6. Reading the accelerometer (`readAccelerometer()`)

This works exactly like reading the gyro, just starting at a different register. Each axis is a **16-bit number** split across 2 registers, a "high" byte and a "low" byte:

| Register | Holds |
|---|---|
| `0x3B` | X high byte |
| `0x3C` | X low byte |
| `0x3D` | Y high byte |
| `0x3E` | Y low byte |
| `0x3F` | Z high byte |
| `0x40` | Z low byte |

The code only says `0x3B`. We tell the sensor "start at `0x3B`, give me 6 bytes", and it hands over `0x3B` through `0x40` in order on its own (a **burst read**). Reading them all at once also makes sure all 6 bytes come from the same reading.

```cpp
Wire.beginTransmission(MPU);
Wire.write(0x3B);             // start at the first accelerometer register
Wire.endTransmission(false);  // keep the bus open, we're about to read
Wire.requestFrom(MPU, 6, true);
```

### Gluing the two bytes back together

```cpp
int16_t Accel_raw_X = Wire.read() << 8 | Wire.read();
```

`<< 8` shifts the high byte 8 spots to the left to make room, and `|` drops the low byte into the empty spot.

Example: the sensor is sitting flat, and Z sends high byte = `0x10`, low byte = `0x00`

```
0x10 << 8     = 0x1000
0x1000 | 0x00 = 0x1000  = 4096
```

Storing it in an `int16_t` (a signed 16-bit integer) means negative readings come out right too. If you flip the sensor upside down, Z reads about **-4096**.

---

## 7. Turning raw numbers into g

The raw number is in **LSB** (Least Significant Bit, the smallest step the sensor can measure). The datasheet (section 4.17) tells us how many LSB make 1 g, depending on the range:

| Range | LSB per g |
|---|---|
| ±2 g | 16384 |
| ±4 g | 8192 |
| **±8 g (what we use)** | **4096** |
| ±16 g | 2048 |

So we divide:

```
accel = raw / 4096

 4096 / 4096 =  1.00 g   (flat on a desk, Z axis)
-2048 / 4096 = -0.50 g
```

The units work out: LSB ÷ (LSB per g) = g.

**Why ±8 g?** Just sitting still only ever shows 1 g, but a plane bouncing on landing or pulling a hard turn can go well past 2 g. ±8 g gives plenty of headroom so the sensor doesn't max out. The tradeoff is each step is a bit bigger, so it's slightly less precise.

---

## 8. Calibration (`calculate_IMU_error()`)

Like the gyro, the accelerometer isn't perfect. Sitting flat, it might say X = 0.02 g and Y = -0.01 g when both should be exactly 0. That's the **error** (or **bias**).

So right after startup we:

1. Take **200 readings**, 1 ms apart (about 200 ms total).
2. Average them for each axis.
3. Save that average as `accelErrorX`, `accelErrorY`, `accelErrorZ`.

Then every reading in `loop()` gets the error subtracted:

```cpp
accelX -= accelErrorX;
```

### Why Z is different

There's one twist compared to the gyro. When the gyro sits still, every axis *should* read 0, so the whole average is error. When the accelerometer sits flat, **Z should read 1 g** because gravity is pulling on it. If we subtracted the whole average, we'd be subtracting gravity itself, and Z would read 0 all the time.

So for Z, the error is only the part *beyond* 1 g:

```cpp
accelErrorZ = (accelErrorZ / calibration_steps - 1.0);
```

Example: Z averages 1.03 g while flat. The error is 1.03 - 1.0 = 0.03 g, and that's all that gets subtracted.

**The sensor has to be flat and completely still during calibration.** The code assumes "flat" means X = 0, Y = 0, Z = 1 g. If the board is tilted while it calibrates, the program treats that tilt as error and removes it, so that tilted position becomes the new "zero". If you bump it, the bump gets baked into every reading after that.

---

## 9. The trig (turning g into angles)

This is the main part of the program.

### Roll

```cpp
accelAngleX = atan2(accelY, accelZ) * 180 / PI;
```

When you roll the sensor, gravity moves from the Z axis over to the Y axis. Picture a right triangle where gravity is the long side, Z is one leg and Y is the other leg. The roll angle is the angle between gravity and Z:

```
tan(roll) = Y / Z     so     roll = atan(Y / Z)
```

Example: rolled 30°. Then Y = sin(30°) = 0.5 g and Z = cos(30°) = 0.866 g.

```
atan(0.5 / 0.866) = atan(0.577) = 30°
```

### Pitch

```cpp
accelAngleY = atan2(accelX, sqrt(square(accelZ) + square(accelY))) * 180 / PI;
```

Pitch is the same idea, but with X. The difference is the bottom of the fraction: instead of just Z, it uses `sqrt(Z² + Y²)`.

That's because if the plane is rolled *and* pitched at the same time, some of the gravity that "belongs" to Z has leaked over into Y because of the roll. `sqrt(Z² + Y²)` adds those two back together (Pythagoras), so the pitch comes out right no matter how much the plane is rolled.

Example: pitched 20°, not rolled. X = sin(20°) = 0.342 g, Y = 0, Z = cos(20°) = 0.940 g.

```
atan(0.342 / sqrt(0.940² + 0²)) = atan(0.364) = 20°
```

### Why `atan2` instead of `atan`

`atan(Y / Z)` has two problems:

1. **Dividing by zero.** If the sensor is on its side, Z = 0, and Y / 0 is undefined.
2. **It can't tell which way is up.** `atan(0.5 / 0.866)` and `atan(-0.5 / -0.866)` give the same answer, even though the second one means the sensor is upside down.

`atan2(Y, Z)` takes the top and bottom separately instead of dividing first. That avoids the divide by zero, and since it can see the sign of each one, it knows which quadrant you're in. It gives answers from -180° to 180°.

### Radians to degrees

`atan2` gives its answer in **radians**. A full circle is 2π radians or 360°, so to convert:

```
degrees = radians × 180 / π
```

That's the `* 180 / PI` at the end of each line.

---

## 10. Timing

The program reads the sensor every 1 ms using `micros()`, the same way the gyro program does:

```cpp
currentTime = micros();
if(currentTime - previousTime < SAMPLE_PERIOD_US){
    return;
}
```

If 1000 µs haven't passed yet, `return;` ends this run of `loop()` early and it starts over.

Unlike the gyro, the accelerometer math doesn't use the time between readings at all, since each reading is its own angle. The 1 ms timing is still useful though, because it keeps the readings evenly spaced, and later on when this gets combined with the gyro, both will need to run at the same rate.

It only prints every 20 ms (50 times a second). Sending text to the computer is slow, and printing every 1 ms would make the loop fall behind (the gyro documentation has the full explanation).

---

## 11. Running it

1. Plug in the Nano.
2. In PlatformIO, pick the **`mpu6050_accel_test`** environment and upload.
3. Open the Serial Monitor and set it to **115200 baud** (if it's set to anything else you'll just get random symbols).
4. **Leave the sensor flat and still for the first second** while it calibrates.
5. Tilt it around and watch the numbers change. Roll it 90° onto its side and roll should read close to ±90.

---

## 12. Known limitation: it can't tell gravity apart from movement

The accelerometer doesn't actually measure gravity. It measures **all** acceleration, and gravity just happens to be the one that's always there. This program assumes gravity is the *only* thing it's feeling, which is only true when the sensor isn't speeding up, slowing down or turning.

Try it: hold the sensor flat and slide it quickly across the desk. The angles jump around, even though you never tilted it. On a plane, the motor vibrating, the plane speeding up, or a hard turn will all mess with the angles the same way.

This is the opposite of the gyro's problem:

| | Short term (a fraction of a second) | Long term (minutes) |
|---|---|---|
| **Gyro** | Smooth and accurate | Drifts away |
| **Accelerometer** | Noisy, thrown off by movement | Stays correct, no drift |

So the plan is to combine them. The gyro handles the quick changes, and the accelerometer slowly pulls the angle back to the truth so the gyro can't drift away. That's called a **complementary filter**, and it's the next step after this test.

---

## 13. Quick reference

| Name in code | Value | What it means |
|---|---|---|
| `MPU` | `0x68` | I2C address of the sensor |
| `ACCELEROMETER_SENSITIVITY` | `4096` | LSB per g at ±8 g |
| `SAMPLE_PERIOD_US` | `1000` | Read the accelerometer every 1000 µs (1 ms) |
| `PRINT_PERIOD_MS` | `20` | Print every 20 ms |
| `calibration_steps` | `200` | Readings averaged during calibration |
| `accelX/Y/Z` | | Current acceleration in g |
| `accelErrorX/Y/Z` | | Calibration error, subtracted from every reading |
| `accelAngleX` | | Roll in degrees |
| `accelAngleY` | | Pitch in degrees |

| Function | What it does |
|---|---|
| `readAccelerometer()` | Reads registers `0x3B` to `0x40` and turns them into g |
| `calculate_IMU_error()` | Averages 200 flat, still readings to find the error on each axis (with 1 g taken out of Z) |
