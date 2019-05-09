# morph_hw

The morph_hw package provides the robot driver for creating a differential drive mobile base for the Modular Open Robotics Platform for Hackers project

## Hardware setup

- Both wheels are connected the same way, inversion of speed for right wheel is done in code
- The phase connections : 1 - 2 - 3 : green - yellow - blue; 1 - 2 - 3 as seen left-to-right from top view of motor controller with phase wires at the bottom
- When testing motor : motor should turn clockwise at positive duty cycle

Note: if needed you can inverse the motor direction either in the motor controller using the VESC tool or by changing the left_wheel_factor or right_wheel_factor in the launch file

## Motor setup

Setup the motor using the latest VESC tool motor setup wizard.

## Encoder or hall sensors

When using an encoder in the motor controller (configured in the VESC tool) set the parameter `rotor_position_source` to `encoder`. If you are not using an external encoder and only using hall sensors, set the `rotor_position_source` to `none`.

## Calculate IKV value

- Start VESC Tool and connect to the motor controller for the left wheel
- Open 'Realtime data' and start realtime data collection with 'RT' button
- Set duty cycle at 0.2
- Wait until the graph is resized to get a better view of the average ERPM
- Write down the ERPM at 0.2 duty cycle (look at the center value of the Y-axis of the graph indicates the average ERPM)
- Write doen the battery voltage

Calculate the IKV as follows:

```
IKV = ERPM / (Vin * poles * 2 * duty_cycle)}
```

- Repeat the same for the right wheel
- Apply the IKV values in the morph.launch file
