% last update: 17/6/14

%run this file to initialize variables used in Simmulink
%LinearizedQuadModel.slx


%%

%define constants
m = 0.53; % [kg]
g_ = 9.806; % [m/s^2]
b_ = 3.13E-5; %[N.s2] thrust factor in hover
Omt = 2*b_*sqrt((m*g_)/(4*b_)); %trim value for Omega_sp

%motor constants
a_ = 0.936;
Tau = 0.178;

l = 0.232; %[m] arm length
Iyy = 6.228E-3;	%// [kg.m2] inertia

%***** constants of the linear curve Omega=f(bin) *****
slo=2.7542; % slope (of the linear curve om=f(bin))
shi=3.627;  % shift



%% height dynamics

%plant states: z, z_dot
%input: U1

A = [0 1; 0 0]; %height dynamics
B = [0;1/m];
C = eye(2);
D = zeros(2,1);

OL = ss(A,B,C,D);

Q(1,1) = (1/5)^2; %penalize on position (5 m is the maximum allowable position error)
Q(2,2) = (1/3)^2; %penalize on velocity (3m/s is the quad's climb rate)

R = (1/8.5)^2; %max U1 is 8.5 if max allowable prop speed is 260rad/s. (to avoid always max out the prop speed for faster response)

Klqr = lqr(OL, Q, R);
CL = ss(A - B*Klqr, B, C, D);
% figure
% subplot(2,1,1); pzmap(OL, 'r', CL, 'b');
% legend('open loop poles', 'closed loop poles'); title('Case 2: 1st order motor + Height dynamics, linmod,(3 poles)')

%Augment state space representation to take into account intergral or error in z
A_aug = [A zeros(2,1);
         -1   zeros(1,2)];  %adding one feedback term (error_z)
B_aug = [B;
         zeros(1,1)];
C_aug = [C zeros(2,1);
         zeros(1,2) 1]; %outputing z, z_dot and dot_error_z only
H = [zeros(2,1);
         1] ; %this is matrix that takes into account the reference input
            
Q2 = [Q zeros(2,1);
     zeros(1,2)  0.1];
Klqr_i = lqr(A_aug,B_aug, Q2, R);
OL_i = ss(A_aug, B_aug, C_aug, zeros(3,1)); %open loop system
CL_i = ss(A_aug - B_aug*Klqr_i, H, C_aug, zeros(3,1));  %new state space system that includes reference input.

% subplot(2,1,2); pzmap(OL_i, 'r', CL_i, 'b');
% legend('open loop poles', 'closed loop poles'); title('Case 2: 1st order motor + Height dynamics, linmod, LQR with integral (4 poles)');


%simulate the tracking with LQR integral
%  t = 0:0.01:20; %simulation time
% r = ones(size(t)); %reference input in z
% figure
% lsim(CL_i, r, t); title('Case 2 (linmod) step response simulation'); %simulation shows that the system tracks reference input. system is STABLE. 
% ylabel('Error_z    ,   z_ dot, [m/s]   ,   Height, z, [m]');


%% LQR Control for theta and phi
%added 11/6/14

% controller for phi is the same of for theta, coz quad's symmetry.

% states = [phi phi_dot] or [theta theta_dot]
% plant input = U2 or U3

A_2 = [0 1;
       0 0];
B_2 = [0;
       l/Iyy]; %Ixx is equal to Iyy in the original model. 
C_2 = eye(2);
D_2 = zeros(2,1);

OL_2  = ss(A_2, B_2, C_2, D_2);

Q_2(1,1) = (1/0.785)^2;
Q_2(2,2) = (1/3)^2;

R_2 = (1/0.5)^2;    

Klqr_2 = lqr(A_2, B_2, Q_2, R_2);
CL_2 = ss(A_2 - B_2*Klqr_2, B_2, C_2, D_2);

% figure
% subplot(2,1,1); pzmap(OL_2, 'r', CL_2, 'b');
% legend('open loop poles', 'closed loop poles'); title('Pole locations for pitch and x-translation channel')

%Augmented state space representation 
A_aug2 = [A_2 zeros(2,1);
         -1   zeros(1,2)];  

B_aug2 = [B_2;
         zeros(1,1)];
C_aug2 = [C_2 zeros(2,1);
         zeros(1,2) 1]; 
H2 = [zeros(2,1);
         1] ; 
            
Q_2i = [Q_2 zeros(2,1);
     zeros(1,2)  1];
Klqr_i_2 = lqr(A_aug2,B_aug2, Q_2i, R_2);
OL_i_2 = ss(A_aug2, B_aug2, C_aug2, zeros(3,1)); %open loop system
CL_i_2 = ss(A_aug2 - B_aug2*Klqr_i_2, H2, C_aug2, zeros(3,1));  %new state space system that includes reference input.

% subplot(2,1,2); pzmap(OL_i_2, 'r', CL_i_2, 'b');
% legend('open loop poles', 'closed loop poles'); title('LQR with integral');


% %simulate the tracking with LQR integral
%   t = 0:0.01:20; %simulation time
%  r = ones(size(t)); %reference input in z
% figure
% lsim(CL_i_2, r, t); title('Step response in pitch'); %simulation shows that the system tracks reference input. system is STABLE. 
% ylabel('Error_theta    ,   theta_ dot, [m/s]   ,   Pitch angle, theta, [m]');



%% x and y position tracking  controller (backstepping & LQR)
% x LQR control gain results in pitch_demand
%y LQR control gain results ni roll_demand

%states = [x x_dot];
%control input = U1

%augment using backstepping. new system: 
%states = [xbar x_dotbar]; where xbar = xref - x
%control input = Uxbar; where Uxbar = Ux*(1/m)U1
%where Ux = cos(theta)cos(phi)  OR Uy = -sin(phi)

Ax = [0 1;
     0 0];
Bx = [0;
      1];
Cx = eye(2);
Dx = zeros(2,1);
 
Qx(1,1) = (1/50)^2; 
Qx(2,2) = (1/20)^2;

Rx = (1/3.5)^2;

Klqr_x = lqr(Ax, Bx, Qx, Rx);
CLx = ss(Ax - Bx*Klqr_x, Bx, Cx, Dx);
% pzmap(CLx);

A_augx = [Ax zeros(2,1);
         -1   zeros(1,2)];  %adding one feedback term (error_x)

B_aug2x = [Bx;
         zeros(1,1)];
C_aug2x = [Cx zeros(2,1);
         zeros(1,2) 1]; %outputing x, x_dot and dot_error_x_dot only
     
Hx = [zeros(2,1);
         1] ; 
            
Q_xi = [Q_2 zeros(2,1);
     zeros(1,2)  0.1];
 
Klqr_i_x = lqr(A_augx,B_aug2x, Q_xi, Rx);
OL_i_x = ss(A_augx, B_aug2x, C_aug2x, zeros(3,1)); %open loop system
CL_i_x = ss(A_augx - B_aug2x*Klqr_i_x, Hx, C_aug2x, zeros(3,1)); 


