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
Continuous_lag_controller = tf([0.3597 1],[0.4554 1])
motor_controlled = motor_response_tf*Continuous_lag_controller*0.2725
Continuous_motor_controlled_feedback = feedback(motor_controlled,1)


stepplot(no_motor_controller,Continuous_motor_controlled_feedback)
title('Motor and Controller with Unity Feedback Simulation')
legend('no motor controller','Continuous motor controlled feedback')
grid on
figure;
stepinfo(Continuous_motor_controlled_feedback)
pole(Continuous_motor_controlled_feedback)
bode(no_motor_controller,Continuous_motor_controlled_feedback)
title('No Controller vs Controlled Frequency Response')
legend('no motor controller','Continuous motor controlled feedback')
grid on
figure;
%}

%***Continuous to discrete conversion***
%Convert continuous controller into discrete controller using the ZOH
%method
ZOH_sampling_1000_ms =c2d(Continuous_lag_controller,1)
ZOH_sampling_500_ms =c2d(Continuous_lag_controller,0.5)
ZOH_sampling_100_ms =c2d(Continuous_lag_controller,0.1)
%Convert continuous controller into discrete controller using the Tustin
%method
tustin_sampling_1000_ms = c2d(Continuous_lag_controller,1,'tustin')
tustin_sampling_500_ms = c2d(Continuous_lag_controller,0.5,'tustin')
tustin_sampling_100_ms = c2d(Continuous_lag_controller,0.1,'tustin')
%***Graph different controllers***
stepplot(Continuous_lag_controller,tustin_sampling_1000_ms,tustin_sampling_500_ms,tustin_sampling_100_ms)
title('Continuous Lag Controller Response vs Discrete Tustin Controllers @ Different Sample Times')
legend('Continuous lag controller','tustin sampling 1000 ms','tustin sampling 500 ms','tustin sampling 100 ms')
grid on
figure;
bode(Continuous_lag_controller,tustin_sampling_1000_ms,tustin_sampling_500_ms,tustin_sampling_100_ms,ZOH_sampling_1000_ms,ZOH_sampling_500_ms,ZOH_sampling_100_ms)
title('Continuous Lag Controller vs Discrete Tustin Controller vs Discrete ZOH Controller')
legend('Continuous lag controller','tustin sampling 1000 ms','tustin sampling 500 ms','tustin sampling 100 ms','ZOH sampling 1000 ms','ZOH sampling 500 ms','ZOH sampling100 ms')
grid on
figure;

%***Continuous to discrete conversion for motor transfer function using Tustin method***
motor_tf_sampling_1000_ms =c2d(motor_response_tf,1,'tustin')
motor_tf_sampling_500_ms =c2d(motor_response_tf,0.5,'tustin')
motor_tf_sampling_100_ms =c2d(motor_response_tf,0.1,'tustin')

%***Graphing discrete system responses***
Discrete_motor_controlled_100ms = motor_tf_sampling_100_ms*tustin_sampling_100_ms*0.2725
Discrete_motor_controlled_feedback_100ms = feedback(Discrete_motor_controlled_100ms,1)

Discrete_motor_controlled_500ms = motor_tf_sampling_500_ms*tustin_sampling_500_ms*0.2725
Discrete_motor_controlled_feedback_500ms = feedback(Discrete_motor_controlled_500ms,1)

Discrete_motor_controlled_1000ms = motor_tf_sampling_1000_ms*tustin_sampling_1000_ms*0.2725
Discrete_motor_controlled_feedback_1000ms = feedback(Discrete_motor_controlled_1000ms,1)

stepplot(Discrete_motor_controlled_feedback_1000ms,Discrete_motor_controlled_feedback_500ms,Discrete_motor_controlled_feedback_100ms,Continuous_motor_controlled_feedback)
title('Continuous Motor Response Vs Discrete Motor Response @ Different Sampling Times')
legend('Discrete motor controlled feedback 1000ms','Discrete motor controlled feedback 500ms','Discrete motor controlled feedback 100ms','Continuous motor controlled feedback')
grid on
