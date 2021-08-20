# Quadrotor_testbed
My Bsc graduation project

The purpose of the project was angle control of a stationary 4-DOF (non-hovering) quadrotor system. The main system runs on an STM32F7 mc board, running C real-time executing sensor commnication, motor speed (PWM) and the PID algorithms. The PID constants were tuned empirically (via manual tuning method). Reference angle is given from a joystick with running python-ROS script externally from another pc. The body frame is majorly made of makeblock mechanical parts

<p align="center">
 <img src="./aero.jpg" scale=".5">
</p>


Demo video:
https://youtu.be/svHYbfm_wV0

More details are available on the thesis
https://drive.google.com/file/d/1CeJ5bQGfB9dTfHgFVUi1hLkSoSA0svkV/view?usp=sharing
