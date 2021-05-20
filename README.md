# SIXT33N

When I was 4 years old, I told my mom I want to design cars when I'm older. At 22 years old, I created my first voice-controlled robot car in my Designing Information Devices and Systems II (EECS 16B) class. 

SIXT33N was a semester-long project, split up into 4 major components: front-end circuitry, system ID, closed-loop control, and classification.

1. Front-end circuitry consisted of building a low-pass filter in order to attenuate frequencies higher than the normal speaking frequency range. We also built a motor controller circuit using BJTs to act as a switch while also amplifying our 3.3V signal to power our 9V motors. My first try was a terrible nest of wires so I spent an entire evening cleaning up the circuit and making it neat!

2. System ID determines the optimal system operating point because our motors move at different velocities. We used linear regression to model the change in input to change in velocity and the constant offset in velocity. 

3. Closed-loop control allows SIXT33N to drive straight. It takes in input at each timestep and feeds it back into the system. Mathematically, it allows us to pick k-values that set our system's eigenvalues to be stable.

4. Classification lets us give SIXT33N commands to "Go", "Slow down there boy", "Turn left", and "Take a right". We learned how Singular Value Decomposition (SVD) and Principal Component Analysis (PCA) is used decompose our signal into its principal components and then project it onto our matrix of already trained and classified values. This projection allows our system to classify the user's spoken words as any of the 4 commands or as noise.

I learned so much from this project. It was incredibly rewarding to learn the theory in lecture and then directly apply it in hands-on lab. I am very thankful to have such an opportunity. Thank you to my EECS 16B TAs Steven, Martin, and Han. Thank you to my labmates Clay, William, and Kimberly. I'm super excited for what I'll learn in the future!

## [Here](https://drive.google.com/file/d/1dEMNnz95w1OrBTrmgmzlXDoxDUSq32ZZ/view?usp=sharing) is a demo of "Go" for SIXT33N!

My circuit: 

![Circuit](Circuit.png)
