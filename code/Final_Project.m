clear;

cum_error = zeros(6,1);
Kp = 0.75*eye(6);
Ki = 0*eye(6);

dt = 0.01;

Tb0 = [1, 0, 0, 0.1662;
        0, 1, 0, 0;
        0, 0, 1, 0.0026;
        0, 0, 0, 1];

M = [1,0,0,0.033;
    0,1,0,0;
    0,0,1,0.6546;
    0,0,0,1];

max_speed = 60;

B1=  [0,0,1,0,0.033,0];
B2 = [0,-1,0,-0.5076,0,0];
B3 = [0,-1,0,-0.3526,0,0];
B4 = [0,-1,0,-0.2176,0,0];
B5 = [0,0,1,0,0,0];
Blist =[B1',B2',B3',B4',B5'];

state = [-0.2 0 -1 0.3 -0.15 -1.9 -0.15 0.15 0 0 0 0 0];
state_ref = [-0.2 -0 -1.2 0.8 -0.15 -1.9 -0.15 0.15 0 0 0 0 0];
% state = [0,0,0,0,0,0,0,0,0,0,0,0];
phi = state_ref(1);
x_b = state_ref(2);
y_b = state_ref(3);

Tsb = [cos(phi), -sin(phi), 0, x_b;
        sin(phi), cos(phi),  0  y_b;
        0,        0,         1, 0.0963;
        0,        0,         0, 1];

T0e = FKinBody(M, Blist, state_ref(4:8)');

X = Tsb*Tb0*T0e;

% Tse_initial = [0, 0, 1, 0;
%                0, 1, 0, 0;
%               -1, 0, 0, 0.5;
%                0, 0, 0, 1];

Tsc_initial = [1, 0, 0, 0.5;
              0, 1, 0, 0;
              0, 0, 1, 0.025;
              0, 0, 0, 1];

Tsc_final = [0, 1, 0, 1;
            -1, 0, 0, -0.5;
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
[ref_trajectory, csv_file] = Trajectory_Generator(X, Tsc_initial, Tsc_final, Tce_grasp, Tce_standoff, k);
[numRows, ~] = size(ref_trajectory);
Wrapper_output = [];
controls1 =[];
X_ERROR = [];

for i = 1:numRows-1

    traj_d = ref_trajectory(i,:);
    traj_d_next = ref_trajectory(i+1,:);

    Xd = VectorToTransform(traj_d(1:12));
    Xd_next = VectorToTransform(traj_d_next(1:12));

    [Twist,Vel, X_error,cum_error] = FeedbackControl(state, X, Xd, Xd_next, Kp, Ki, dt,cum_error);
    cum_error = cum_error + X_error*dt;
    X_ERROR = [X_ERROR,X_error];
    controls = [Vel(5:9)', Vel(1:4)'];
  
    state = NextState(state, controls, dt, max_speed);

    phi = state(1);
    x_b = state(2);
    y_b = state(3);
    Tsb = [cos(phi), -sin(phi), 0, x_b;
           sin(phi), cos(phi),  0, y_b;
           0,        0,         1, 0.0963;
           0,        0,         0, 1];
    T0e = FKinBody(M, Blist, state(4:8)');
    X = Tsb * Tb0 * T0e;  % Recalculate X

    Wrapper_output = [Wrapper_output; [state,ref_trajectory(i,13)]];
    % X = Xd;
end

writematrix(Wrapper_output, "Wrapper_output.csv")

figure;
hold on;  % all plots are on the same figure
time = (1:numRows-1) * dt/k;  % Create a time vector 

for i = 1:6
    plot(time, X_ERROR(i, :));
end

title('Error Components Over Time');
xlabel('Time (seconds)');
ylabel('Error');
legend('X_err1', 'X_err2', 'X_err3', 'X_err4', 'X_err5', 'X_err6');
hold off;

