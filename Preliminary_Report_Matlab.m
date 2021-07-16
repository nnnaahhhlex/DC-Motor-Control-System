clc
clear all
kt = 2.29*10^-2; % Torque Constant UNIT: N-m/A
ke = 2.29*10^-2; % Back-EMF Constant UNIT: V/rad/s
R = 0.71; % Unit: Ohm
L = 0.66*10^-3; % Inductance UNIT: Conversion mH ---> H 
f = 8.5*10^-4; % Damping Constant UNITS: N-m-s
J = ((7.1*10^-6)*5.9^2)+(1.4*10^-4)+(2.6*10^-4); % Total System Inertia UNITS: kg-m^2

%***Motor Response***

field_block = tf([1],[L R]) %Modeling the field control
armature_block = tf([1],[J f])%Modeling the armature control
%Combined the torque... 
%...constant,field block and armature block into a single block.
combined_blocks = field_block*armature_block*kt; 
gear_reduction = tf([1],[5.9 0])
%Built the EMF feedback response and combined an integrator...
%...(gear reduction) to get the position response. An amp gain is added.
motor_response_tf = feedback(combined_blocks,ke)*gear_reduction*2.5 
%... A closed loop motor response was built. Without a controller, the
%response is underdamped.
no_motor_controller = feedback(motor_response_tf,1)

%***Motor Response Charts***
%{
stepplot(motor_response_tf)
title('Open Loop Motor Response')
figure;
stepplot(no_motor_controller)
title('Closed Loop Motor Response')
grid on
figure;
stepinfo(no_motor_controller)
pole(no_motor_controller)
bode(no_motor_controller)
title('Closed Loop Motor Response Bode Diagram')
grid on
figure;
%}

%***Lag Comp***

%A higher phase margin is required to reduce the overshoot. This is
%...achieved because the damping ratio is reduced. A gain value is added to
%...reduce the overshoot further.
lag = tf([0.3597 1],[0.4554 1])
motor_controlled = motor_response_tf*lag*0.2725
motor_controlled_feedback = feedback(motor_controlled,1)

stepplot(no_motor_controller,motor_controlled_feedback)
title('Motor and Controller with Unity Feedback Simulation')
grid on
stepinfo(motor_controlled_feedback)
pole(motor_controlled_feedback)
figure;
bode(no_motor_controller,motor_controlled_feedback)
grid on
%}

%***Continuous to discrete conversion***

sampling_1000_ms =c2d(lag,1)
sampling_500_ms =c2d(lag,0.5)
sampling_100_ms =c2d(lag,0.1)


%sampling_500_ms =c2d(motor_controlled_feedback,0.5)
%sampling_1000_ms =c2d(motor_controlled_feedback,1)

stepplot(sampling_100_ms,sampling_500_ms,sampling_1000_ms,lag)
title('Continuous VS Discrete Sampling')
%}


