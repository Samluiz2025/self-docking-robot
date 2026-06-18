# Wiring Reference

## Raspberry Pi 4 GPIO Map (BCM numbering)

| GPIO | Pin | Component         | Function              | Notes                              |
|------|-----|-------------------|-----------------------|------------------------------------|
| 17   | 11  | L298N IN1         | Left motor dir A      | Digital output                     |
| 18   | 12  | L298N IN2         | Left motor dir B      | Digital output                     |
| 22   | 15  | L298N IN3         | Right motor dir A     | Digital output                     |
| 23   | 16  | L298N IN4         | Right motor dir B     | Digital output                     |
| 12   | 32  | L298N ENA         | Left motor PWM        | Hardware PWM                       |
| 13   | 33  | L298N ENB         | Right motor PWM       | Hardware PWM                       |
| 5    | 29  | Left encoder A    | Interrupt input       | 3.3V signal from encoder           |
| 6    | 31  | Left encoder B    | Input                 |                                    |
| 19   | 35  | Right encoder A   | Interrupt input       |                                    |
| 26   | 37  | Right encoder B   | Input                 |                                    |
| 24   | 18  | HC-SR04 TRIG      | Ultrasonic trigger    | Digital output                     |
| 25   | 22  | HC-SR04 ECHO      | Ultrasonic echo       | Input — use voltage divider!       |
| 21   | 40  | IR receiver OUT   | Dock beacon detector  | TSOP38238 output, PUD_UP           |
| 2    | 3   | INA219 SDA        | I2C data              | I2C address 0x40                   |
| 3    | 5   | INA219 SCL        | I2C clock             |                                    |
| USB  | —   | Logitech C270     | /dev/video0           | USB 2.0                            |
| USB  | —   | RPLIDAR A1        | /dev/ttyUSB0          | USB — may need: chmod 666 /dev/ttyUSB0 |

## Power Rail

```
3S Li-ion (11.1V) → BMS → [12V rail]
                               ├── L298N 12V input (motors)
                               └── L298N 5V out → NOT recommended for RPi
                                   (use separate USB-C 5V 3A adapter for RPi)
```

## Voltage Divider for HC-SR04 ECHO

The HC-SR04 ECHO pin outputs 5V. The RPi GPIO is 3.3V tolerant only.

```
ECHO ──[1kΩ]──┬── GPIO 25 (RPi)
              [2kΩ]
               │
              GND
```

Output voltage = 5V × 2kΩ / (1kΩ + 2kΩ) = 3.33V ✓
