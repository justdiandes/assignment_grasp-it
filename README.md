Operational Control with iCub
=============================

[![Gitpod](https://gitpod.io/button/open-in-gitpod.svg)](https://gitpod.io/from-referrer)

# Prerequisites
By now, you should be an expert of the following components :wink::
- [Joint Control](http://www.icub.org/doc/icub-main/icub_motor_control_tutorial.html).
- [Cartesian Control](http://www.icub.org/doc/icub-main/icub_cartesian_interface.html).
- [Gaze Control](http://www.icub.org/doc/icub-main/icub_gaze_interface.html).

# Assignment
We want you to develop a module that employs the `Joint Interface`, the `Cartesian Interface`
and the `Gaze Interface` to accomplish the following tasks:

1. Make iCub **look down at the table**.
1. **Detect the object** lying on the table.
1. Retrieve the **object position** in the Cartesian domain.
1. Let iCub **look at the object**.
1. Based on the object position, **select the best hand**.
1. **Control the iCub fingers** to achieve suitable grasp configurations.
1. Let iCub **approach the object** to enable a top grasp.
1. Ask iCub to **grasp the object**.
1. **Lift the object**.

This assignment is peculiar in that **we can interface the simulator and the real robot indifferently**,
being the code capable of dealing with object location retrieval in both conditions, transparently to the user.

The outcome should look like the animation below with the simulator:

<p align="center">
  <img src="/assets/grasp-it.gif"/>
</p>

---

Some of the points reported above have been already addressed in the code (e.g. [**object detection and location retrieval**](./src/helpers.h#L24)),
so you need to **fill in the missing gaps** highlighted by the comment `// FILL IN THE CODE` in the [**`src/main.cpp`**](./src/main.cpp) module.

⚠ Don't speed up the movements by reducing the trajectory time of the operational controllers: they're already all set to work with the real robot.

Once done, you can test your code in two ways along with the simulator:

1. **Manually**: running the _yarpmanager scripts_ provided from within [**app/scripts**](./app/scripts) and yielding corresponding commands to the _module rpc port_.
1. **Automatically**: [running the script **test.sh**](https://github.com/vvv-school/vvv-school.github.io/blob/master/instructions/how-to-run-smoke-tests.md) in the **smoke-test** directory. Take into account these important points:
    1. We use a **timeout of _240 seconds_** to check the status of rpc communication, meaning that you have _240 seconds_ max to accomplish each rpc command.
    1. When you reply to rpc commands, we assume the robot has **finished the movement**.
    1. The smoke-test will add a random displacement to the initial position of the ball in order to force the use of both hands :wink:

If you pass the test on the simulator, 🕒 **book the robot** 🤖 to get a real experience!

# [How to complete the assignment](https://github.com/vvv-school/vvv-school.github.io/blob/master/instructions/how-to-complete-assignments.md)
