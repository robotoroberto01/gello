# Passive Gello node — electrical review

The intended parts are seven XL330 encoders, a dedicated ESP32-S3 and one SN74AHCT125 from the recorded ten-piece order. The hand's ESP32s, XC330s and regulator remain allocated to the hand builds. There is no verified completed wiring diagram or powered test for this node.

## Interface corrections

The [TI SN74AHCT125 datasheet](https://www.ti.com/lit/ds/symlink/sn74ahct125.pdf) specifies 4.5–5.5 V operation and output levels referenced to that supply. Do not connect a 5 V powered AHCT RX output directly to an [ESP32-S3 input](https://www.espressif.com/sites/default/files/documentation/esp32-s3_datasheet_en.pdf). The receive path needs an appropriate 3.3 V output level. Finalise a rated buffer/level shifter and its schematic before wiring. The transmit enable is active-low on AHCT125: the sketch's DIR HIGH means transmit, so its TX enable needs the corresponding inversion. A comment saying "DIR on enables" is not a circuit.

The [ROBOTIS XL330 manual](https://emanual.robotis.com/docs/en/dxl/x/xl330-m288/) describes a 3.3 V TTL bus compatible with 5 V TTL. That does not make the ESP32 5 V tolerant. Keep grounds common only within the designed low-voltage interface and verify no contention during direction changes.

## Power and operating mode

A host USB port and development board must have a verified current path and available current before supplying the seven encoders. Do not assume every port supplies 1 A or that motor-current setpoints equal the complete USB load. Qualify torque-off startup/inrush, steady current and bus voltage at the farthest servo. If using external regulated 5 V, prevent backfeeding the host USB supply. No spare hand regulator is allocated here.

The default firmware only disables torque and reads encoders. It does not enable current mode or parking. ROBOTIS defines Goal Current as commanded torque/current in current mode; constant current is not position holding. A future hold mode needs a measured current budget, captured position targets, bounded gains, command/readback checks and timeout behaviour.

## Before connection

Confirm servo IDs and baud one servo at a time using a proper commissioning tool. This sketch does not assign IDs; it reads IDs 1–7 at 1 Mbps. Fable's proposed ID-assignment utility is not present. U2D2 is optional if a validated alternative exists and does not provide servo power.

A native test harness covers malformed packet sizes, wrong IDs, CRC failure, alert status, destination capacity and torque-off at boot / `t`. Remaining protocol work includes byte-stuffing support and serial resynchronisation. Compile for the exact ESP32-S3 board and test the complete bus before relying on samples. No hardware was connected or operated during this review.
