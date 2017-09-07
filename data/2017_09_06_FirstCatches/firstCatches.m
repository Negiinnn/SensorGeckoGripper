% Control being used here was active bang-bang.
% While the gripper was idle, commanded current was zero. 
% Thresh hold is based on velocity.
% Experiment was run on a compliant wrist on the white board
% 
% Tuning
% 	changed velocity threshold to dynamixel units of 10 
% 
% Trial 1 - passive control
% 	Grasped object and there was a LITTLE oscillation, still pretty successful though 
% 	Kp = .25 
% 	Kd = .2 
% 
% Trial 2 - passive control again
% 	Changed the position on the white board table since it was scratched 
% 
% Trial 3 - passive control 
% 	Coated the object with paper and coerced a fail out of it from spinning
% 	Reviewed the video and it looked like it had a fair chance --we should look at the force data 
% 
% Trial 4 - active
% 	Cone needs to be changed 
% 	
% 
