state = [-0.2 -0.6 -0.01 0.3 -0.15 -1.9 -0.15 0.15 0 0 0 0 0];
% state = [0,0,0,0,0,0,0,0,0,0,0,0];
phi = state(1);
x_b = state(2);
y_b = state(3);

Tsb = [cos(phi), -sin(phi), 0, x_b;
        sin(phi), cos(phi),  0  y_b;
        0,        0,         1, 0.0963;
        0,        0,         0, 1];

T0e = FKinBody(M, Blist, state(4:8)');

Tse_initial = Tsb*Tb0*T0e;

Tse_initial = [0, 0, 1, 0;
               0, 1, 0, 0;
              -1, 0, 0, 0.5;
               0, 0, 0, 1];

Tsc_initial = [1, 0, 0, 1;
              0, 1, 0, 0;
              0, 0, 1, 0.025;
              0, 0, 0, 1];

Tsc_final = [0, 1, 0, 0;
            -1, 0, 0, -1;
            0, 0, 1, 0.025;
            0, 0, 0, 1];

Tce_grasp = [cos(3*pi/4), 0, sin(3*pi/4), 0;
                0, 1, 0, 0;
                -sin(3*pi/4), 0, cos(3*pi/4), 0;
                0, 0, 0, 1];

Tce_standoff = [cos(3*pi/4), 0, sin(3*pi/4), 0;
                0, 1, 0, 0;
                -sin(3*pi/4), 0, cos(3*pi/4), 0.2;
                0, 0, 0, 1];
k = 3;
[ref_trajectory, csv_file] = Trajectory_Generator(Tse_initial, Tsc_initial, Tsc_final, Tce_grasp, Tce_standoff, k);
% Initialize state and control histories
% state_history = [];
% control_history = [];
% 
% % Initial robot configuration (assuming you have this defined somewhere)
% current_state = [0, 0, 0, zeros(1, 5), zeros(1, 4)];
% 
% % Loop through each step of the trajectory
% for i = 1:size(ref_trajectory, 1) - 1
%     % Current and next desired poses and gripper states from the trajectory
%     Xd = ref_trajectory(i, 1:end-1);
%     Xd_next = ref_trajectory(i+1, 1:end-1);
% 
%     % Convert the desired poses into transformation matrices or use them as needed
%     % You need to reshape Xd and Xd_next back to 4x4 matrices if necessary
%     phi = state(1);
%     x_b = state(2);
%     y_b = state(3);
% 
%     Tsb = [cos(phi), -sin(phi), 0, x_b;
%         sin(phi), cos(phi),  0  y_b;
%         0,        0,         1, 0.0963;
%         0,        0,         0, 1];
% 
% 
%     T0e = FKinBody(M, Blist, state(4:8)');
% 
%     X = Tsb*Tb0*T0e;
% 
%     % Calculate the control input using the FeedbackControl function
%     [twist, u, error] = FeedbackControl(current_state,X, Xd, Xd_next, Kp, Ki, dt);
% 
% 
%     % Update the state using the NextState function
%     current_state = NextState(current_state, [u(5:9)',u(1:4)'], dt,10);
% 
%     % Store the current state and control input for analysis and visualization
%     state_history = [state_history; [current_state,ref_trajectory(i,end)]];
%     control_history = [control_history; V, u, joint_velocities];
% end
% 
% % Generate the final trajectory for visualization or further analysis
% trajectory = state_history;  % This is the simulated trajectory of the robot
% 
% % Optionally, save the state and control histories to files
% writematrix(state_history,'state_history.csv');