# RoboVision
The system first identifies key landmarks on the user's body, such as shoulders, elbows, and wrists. These landmarks are crucial for determining the configuration of
the user's limbs and joints. Once the landmarks are identified, their spatial coordinates (x, y) within the frame are
extracted. These coordinates represent the positions of the landmarks relative to the
frame's origin (usually the top-left corner). For each joint (e.g., shoulder, elbow), reference lines are defined using pairs of
adjacent landmarks. These reference lines represent the orientation of the limb
segments connected by the joint. The angles between the reference lines are then calculated using arctangent
trigonometric functions. There is another feature of the code,” STEPS” that breaks the transition from the starting
position of the servo to the final position into various steps in between which
eventually smooths the movement, to achieve this we use the numpy library, NumPy's
linspace function generates a specified number of evenly spaced values over a
specified range. In this case, it generates steps number of values between start_angle
and end_angle. There is also a use of The sin function is used to create a smooth
transition between these values.
This project represents a pioneering integration of cutting-edge technologies, merging computer vision, real-time human motion tracking, robotics, and IoT to deliver innovative solutions with unparalleled security features and versatile industrial applications.

Upon entering our Python environment or application, users initiate the setup process by positioning external cameras strategically to capture precise movements. Alternatively, a single camera with adequate field of view, spanning from head to knee, suffices. Subsequently, users authenticate via our app/web interface, where credentials undergo stringent validation. A dual-layer security check ensues, incorporating both CAPTCHA verification and facial recognition against stored models. Successful authentication grants access to the system, subject to an additional calibration procedure ensuring optimal accuracy of the robotic arm or other controlled apparatus.

Once calibrated, users can manipulate the system via hand gestures or predefined landmarks, facilitated by real-time gesture tracking. A vital safety mechanism triggers in the event of abrupt gestures exceeding predefined thresholds, swiftly returning the manipulator to a neutral position. This protocol averts potential accidents arising from inadvertent movements, with customizable thresholds tailored to specific operational contexts.

As users interact with the system, our software seamlessly transmits pertinent data to the server for processing. Commands are then relayed to the robotic manipulator's control board, orchestrating precise responses. Alternatively, users can harness high-powered processing units to establish direct connectivity with the server and manipulator, leveraging a shared network infrastructure.

Our system accommodates diverse video feed sources, with options ranging from integrated laptop webcams to external devices such as ESP32 camera modules or mobile devices. Seamless integration necessitates coexistence within the same network, ensuring efficient communication between the controller and video source.

In essence, our project epitomizes a harmonious fusion of advanced technologies, fostering seamless interaction, robust security, and unparalleled versatility in industrial applications.

![image](https://github.com/clenser/RoboVision/assets/100501976/30ff4759-483a-4c03-a0dc-e0dc4c35c9a4)


![image](https://github.com/clenser/RoboVision/assets/100501976/1fb4cb3d-5092-4573-a187-ba358ec3b574)

![image](https://github.com/clenser/RoboVision/assets/100501976/223fa2b5-0a5c-435c-ab5a-4b4df539ea4c)



