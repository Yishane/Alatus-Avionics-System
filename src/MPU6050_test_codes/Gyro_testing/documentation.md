# Gyro Testing: Finding the Plane's Angle with an MPU-6050

This program reads the gyroscope on an MPU-6050, figures out how far the sensor has tilted and turned, and prints the angles to the Serial Monitor. 

The output looks like this:

```
Roll: 0.02  Pitch: -0.01  Yaw: 0.00
Roll: 4.87  Pitch: -0.15  Yaw: 0.03
Roll: 12.41  Pitch: -0.30  Yaw: 0.10
```

All angles are in degrees.

---

## 1. The big idea

Here's the most important thing to understand before touching the code:

**A gyroscope does not tell you your angle. It tells you how fast you're spinning.**

Think about a car. The speedometer says 60 km/h, but it has no idea where you are. If you want to know how far you've gone, you have to take the speed and multiply it by how long you've been driving.

The gyro works the same way. It tells us something like "you're rotating at 30 degrees per second". To get the actual angle, we keep adding up (speed × time) over and over, thousands of times a second. In math this is called **integrating**, and that's the whole job of this program.

For students that have taken or are taking calc, it's the opposite of a derivative, so if dx/dt = 3 (m/s), then taking the integral of that would give you x(t) = 3t this position x as a function of t.


---

## 2. Roll, pitch and yaw

The MPU-6050 measures spin around 3 axes. With the board sitting flat and the X arrow pointing at the nose of the plane:

| Axis | Name | What it looks like on a plane |
|---|---|---|
| X | **Roll** | One wing goes up, the other goes down |
| Y | **Pitch** | Nose goes up or down |
| Z | **Yaw** | Nose turns left or right (like shaking your head "no") |

If the board is mounted a different way on the plane, these names get swapped around, so check which way the arrows on the board point.


---

## 3. Wiring

| MPU-6050 | Arduino Nano |
|---|---|
| VCC | 5V |
| GND | GND |
| SCL | A5 |
| SDA | A4 |

SCL and SDA are the two wires for **I2C**, which is how the Arduino talks to the sensor (more on that in section 5).

---

## 4. What the program does, in order

**Once, when it turns on (`setup()`):**

1. Start the Serial Monitor connection and the I2C bus.
2. **Wake up the MPU-6050.** It starts in sleep mode, so we have to tell it to get up.
3. **Set the gyro range** to ±2000 degrees per second.
4. **Calibrate** (`calculate_IMU_error()`). The sensor has to sit perfectly still for about 200 ms here.
5. Set all the angles to 0. Whatever position the sensor is in right now counts as "zero".

**Indefinitely after that (`loop()`):**

1. Wait until 1 millisecond has passed.
2. Read the gyro (`readGyro()`).
3. Subtract the calibration error.
4. Add the little bit of rotation from this millisecond onto the total angle (trapezoidal rule, section 8).
5. Every 20 ms, print the angles.

---

## 5. How the Arduino talks to the sensor (I2C and registers)

The MPU-6050 has a bunch of numbered storage slots inside it called **registers**. Think of them like school lockers. Each locker has a number (an address), and each one holds a single byte. Some lockers are settings we write to, and some hold sensor readings we read from.

The lockers we use:

| Register | What it's for | What we do |
|---|---|---|
| `0x75` | WHO_AM_I. Holds `0x68`, the sensor's own I2C address | Not used in this program, but handy for checking the sensor is connected |
| `0x6B` | Power management | Write `0x00` to wake it up and use the internal 8 MHz clock |
| `0x1B` | Gyro settings | Write `0x18` to set the range to ±2000 °/s |
| `0x43` to `0x48` | The gyro readings | Read these every millisecond |

(The `0x` just means the number is written in hexadecimal. `0x6B` is 107 in normal numbers.)

The sensor itself also has an address on the I2C bus, `0x68`. That's how the Arduino says "hey, I'm talking to *you*" when there could be several devices on the same two wires.

**Writing a setting** always looks like this:

```cpp
Wire.beginTransmission(MPU);  // "Hey sensor at 0x68"
Wire.write(0x6B);             // "open locker 0x6B"
Wire.write(0x00);             // "put 0 in it"
Wire.endTransmission(true);   // send it and hang up
```

---

## 6. Reading the gyro (`readGyro()`)

Each axis reading is a **16-bit number**, but each register only holds **8 bits** (1 byte). So each axis gets split across 2 registers, a "high" byte and a "low" byte:

| Register | Holds |
|---|---|
| `0x43` | X high byte |
| `0x44` | X low byte |
| `0x45` | Y high byte |
| `0x46` | Y low byte |
| `0x47` | Z high byte |
| `0x48` | Z low byte |

You might notice the code only ever says `0x43`. That's because the MPU-6050 moves to the next locker on its own every time it sends a byte. We tell it "start at `0x43`, give me 6 bytes", and it hands over `0x43` through `0x48` in order. This is called a **burst read**.

It's also the *correct* way to do it. If we read each register one at a time, the sensor could update its readings halfway through, and we'd end up gluing the high byte from one reading to the low byte from the next one (which gives garbage).

### Gluing the two bytes back together

The Gyro gives us a 16-byte number, but in 2 separate 8-bit bytes, so we have to recombine them to get the actual raw gyro data.

Example:

raw = 12, 000 LSB 

binary = 00010111 01110000

High_byte = 00010111

Low_byte = 01110000

We would have to combine the high and low bye in order to get our actual raw gyro data, which we can do:

```cpp
int16_t rawX = (Wire.read() << 8) | Wire.read();
```

`<< 8` shifts the high byte 8 spots to the left to make room, and `|` drops the low byte into the empty spot.

Example: high byte = `0x01`, low byte = `0x48`

```
0x01 << 8    = 0x0100
0x0100 | 0x48 = 0x0148  = 328
```

### Negative numbers

Spinning one way gives a positive number and spinning the other way gives a negative one. The sensor stores negatives using a system called **two's complement**. You don't need to know how it works inside. Just know that storing the result in an `int16_t` (a signed 16-bit integer) makes the Arduino read it correctly. For example, `0xFFD8` comes out as **-40**, not 65496.

---

## 7. Turning raw numbers into degrees per second

The raw number (like 328) isn't in degrees per second yet. It's in **LSB**, which stands for Least Significant Bit. That's the smallest step the sensor can measure, kind of like how the smallest mark on a ruler is 1 mm.

The datasheet tells us how many LSB equal 1 degree per second. It depends on the range we picked:

| Range | LSB per °/s |
|---|---|
| ±250 °/s | 131 |
| ±500 °/s | 65.5 |
| ±1000 °/s | 32.8 |
| **±2000 °/s (what we use)** | **16.4** |

So we divide:

```
rate = raw / 16.4

328 / 16.4 = 20 °/s
-40 / 16.4 = -2.44 °/s
```

The units work out too: LSB ÷ (LSB per °/s) = °/s.

**Why ±2000?** It's the biggest range, so a fast spin (like a crash, or a very enthusiastic roll) won't max out the sensor. The tradeoff is that each step is a bit bigger, so it's slightly less precise for slow movements.

---

## 8. Calibration (`calculate_IMU_error()`)

No sensor is perfect. Even when the MPU-6050 is sitting completely still on a desk, it'll say something like "you're spinning at 0.8 °/s". That fake reading is called the **error** (or **bias**).

That doesn't sound like much, but remember we're adding up the rate forever. 0.8 °/s for one minute is 48 degrees of tilt that never happened.

So right after startup we:

1. Take **200 readings**, 1 ms apart (about 200 ms total).
2. Average them for each axis.
3. Save that average as `gyroErrorX`, `gyroErrorY`, `gyroErrorZ`.

Then every reading in `loop()` gets the error subtracted:

```cpp
gyroX -= gyroErrorX;
```

**The sensor has to be completely still during calibration.** If you bump it, the program thinks the bump is the sensor's normal error and subtracts it from every reading after that. Then the angles drift like crazy.

---

## 9. The trapezoidal rule (turning speed into angle)

This is the "speed × time" part from section 1, done a bit smarter.

If you graph the spin rate over time, the **angle is the area under that graph**. We can't get a perfect area, so we chop the graph into tiny 1 ms slices and add up the area of each slice.

The simple way would be to treat each slice as a rectangle (current rate × time). The trapezoidal rule treats each slice as a **trapezoid** instead, using the rate at the start *and* the end of the slice:

```
area of one slice = (previous rate + current rate) / 2 × time
```

A trapezoid hugs the real curve way better than a rectangle does, especially when the rate is changing quickly.

In the code:

```cpp
angleX += (prevGyroX + gyroX) * 0.5 * elapsedTime;
```

### Example

Last reading: 10 °/s. This reading: 12 °/s. Time between them: 1 ms = 0.001 s.

```
(10 + 12) / 2 × 0.001 = 0.011 degrees
```

So the angle goes up by 0.011°. Do that 1000 times a second and it adds up to real rotation.

After each step, the current rate gets saved as `prevGyroX` so it can be the "previous" one next time.

(One small detail: at startup we take one extra reading to fill in `prevGyroX` before the loop starts. Otherwise the very first trapezoid would start from 0, which isn't true.)

---

## 10. Timing

### Why `micros()` instead of `millis()`

`elapsedTime` is the time between readings, and it goes straight into the angle math. If it's off, every single angle is off.

`millis()` only counts whole milliseconds, which is way too rough when each step is only 1 ms. `micros()` counts in microseconds (millionths of a second), so we get a much more accurate time. We also **measure** the real time between readings instead of just assuming it's exactly 1 ms, since the loop is never perfectly on time.

### Why only print every 20 ms

Sending text to the computer is slow. At 115200 baud, one line like `Roll: 12.41  Pitch: -0.30  Yaw: 0.10` takes about 3 ms to send, which is longer than our whole 1 ms loop. If we printed every loop, we'd fall behind.

So the math still runs every 1 ms, but we only print every 20 ms (50 times a second). That's still way faster than anyone can read.

---

## 11. Running it

1. Plug in the Nano.
2. In PlatformIO, pick the **`mpu6050_test`** environment and upload.
3. Open the Serial Monitor and set it to **115200 baud** (if it's set to anything else you'll just get random symbols).
4. **Leave the sensor still for the first second** while it calibrates.
5. Tilt it around and watch the numbers change.

---

## 12. Known limitation: drift

If you leave the sensor still for a while, you'll notice the angles slowly creep away from where they should be, especially yaw. That's **drift**.

It happens because even after calibration, there's a tiny bit of error left over, and since we add up every reading forever, that tiny error keeps piling up. A gyro alone can't fix this, because it has no idea which way is actually "down".

That's where the accelerometer comes in later. It can sense gravity, so it can correct roll and pitch over time. (Yaw needs something else, like a compass, since gravity doesn't change when you turn left or right.) For this test though, some drift is normal and expected.

---

## 13. Wire.h function cheat sheet

Every I2C function you'd normally use with the MPU-6050, in the order you'd use them. All of them come from `#include <Wire.h>` and get called on the `Wire` object.

The last column says whether this program actually uses it. The two marked "No" aren't needed here, but they're good to know about.

| Function | What it does | Parameters | Returns | Used here? |
|---|---|---|---|---|
| `Wire.begin()` | Starts the I2C bus with the Arduino in charge (the controller). Call it once in `setup()`. | None | Nothing | Yes |
| `Wire.setClock(frequency)` | Sets how fast the I2C bus runs. | `frequency`: in Hz. 100000 is the default, 400000 is "fast mode" (the MPU-6050 can handle it). | Nothing | No |
| `Wire.beginTransmission(address)` | Starts a message to one device. Nothing actually gets sent yet, it just starts filling up a buffer. | `address`: the device's 7-bit I2C address. The MPU-6050 is `0x68` (or `0x69` if its AD0 pin is high). | Nothing | Yes |
| `Wire.write(value)` or `Wire.write(array, length)` | Adds byte(s) to the buffer. The first byte is the register address, and anything after that is the data that goes into that register. | `value`: one byte. Or an array plus its length. | How many bytes got queued | Yes |
| `Wire.endTransmission(stop)` | Actually sends everything in the buffer. | `stop`: `true` (default) sends a STOP and lets go of the bus. `false` keeps the bus so a read can happen right after (a "repeated start"). | A status number: 0 = it worked, 1 = too much data, 2 = no reply to the address, 3 = no reply to the data, 4 = some other error, 5 = timed out | Yes |
| `Wire.requestFrom(address, quantity, stop)` | Asks the device to send bytes back. They land in a receive buffer. | `address`: 7-bit I2C address. `quantity`: how many bytes to read. `stop`: same as above, default `true`. | How many bytes actually came back | Yes |
| `Wire.available()` | Checks how many received bytes are waiting to be read. | None | Number of bytes waiting | No |
| `Wire.read()` | Takes the next byte out of the receive buffer. | None | The byte (0 to 255), or -1 if there's nothing left | Yes |

A useful one for debugging: if the sensor isn't responding, `Wire.endTransmission()` returning **2** means nothing answered at that address. That usually means a loose wire, or that the address is wrong.

---

## 14. Quick reference

| Name in code | Value | What it means |
|---|---|---|
| `MPU` | `0x68` | I2C address of the sensor |
| `GYRO_SENSITIVITY` | `16.4` | LSB per °/s at ±2000 °/s |
| `SAMPLE_PERIOD_US` | `1000` | Read the gyro every 1000 µs (1 ms) |
| `PRINT_PERIOD_MS` | `20` | Print every 20 ms |
| `CALIBRATION_SAMPLES` | `200` | Readings averaged during calibration |
| `gyroX/Y/Z` | | Current spin rate in °/s |
| `prevGyroX/Y/Z` | | Spin rate from the last reading (for the trapezoid) |
| `gyroErrorX/Y/Z` | | Calibration error, subtracted from every reading |
| `angleX/Y/Z` | | Roll, pitch, yaw in degrees |
| `elapsedTime` | | Real time between readings, in seconds |

| Function | What it does |
|---|---|
| `readGyro()` | Reads registers `0x43` to `0x48` and turns them into °/s |
| `calculate_IMU_error()` | Averages 200 still readings to find the error on each axis |
