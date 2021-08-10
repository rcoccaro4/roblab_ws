% Fanuc m20ia forward kinematics

q = zeros(6,1);

dh_params = [   0.525,      q(1),       0.15,   -pi/2;
                0,          q(2)+pi/2,  0.79,   0;
                0,          q(3),       0.15,   pi/2;
                0.86,       q(4)-pi,    0,      pi/2;
                0,          q(5),       0,      -pi/2;
                0.1,        q(6),       0,      0       ];
            
A_1_0 = [cos(dh_params(1,2)) 0 -sin(dh_params(1,2)) (0.15 * cos(dh_params(1,2))); sin(dh_params(1,2)) 0 cos(dh_params(1,2)) (0.15 * sin(dh_params(1,2))); 0 -1 0 0.525; 0 0 0 1];
A_2_1 = [cos(dh_params(2,2)) -sin(dh_params(2,2)) 0 (0.79 * cos(dh_params(2,2))); sin(dh_params(2,2)) cos(dh_params(2,2)) 0 (0.79 * sin(dh_params(2,2))); 0 0 1 0; 0 0 0 1];
A_3_2 = [cos(dh_params(3,2)) 0 -sin(dh_params(3,2)) (0.15 * cos(dh_params(3,2))); sin(dh_params(3,2)) 0 cos(dh_params(3,2)) (0.15 * sin(dh_params(3,2))); 0 -1 0 0; 0 0 0 1];
A_4_3 = [cos(dh_params(4,2)) 0 sin(dh_params(4,2)) 0; sin(dh_params(4,2)) 0 -cos(dh_params(4,2)) 0; 0 1 0 0.86; 0 0 0 1];
A_5_4 = [cos(dh_params(5,2)) 0 -sin(dh_params(5,2)) 0; sin(dh_params(5,2)) 0 cos(dh_params(5,2)) 0; 0 -1 0 0; 0 0 0 1];
A_6_5 = [cos(dh_params(6,2)) -sin(dh_params(6,2)) 0 0; sin(dh_params(6,2)) cos(dh_params(6,2)) 0 0; 0 0 1 0.1; 0 0 0 1];

% Trasnformations to the end effector
A_6_0 = A_1_0 * A_2_1 * A_3_2 * A_4_3 * A_5_4 * A_6_5;
A_6_1 = A_2_1 * A_3_2 * A_4_3 * A_5_4 * A_6_5;
A_6_2 = A_3_2 * A_4_3 * A_5_4 * A_6_5;
A_6_3 = A_4_3 * A_5_4 * A_6_5;
A_6_4 = A_5_4 * A_6_5;

% Rotation Matrix
rotm_0 = tform2rotm(A_6_0);
rotm_1 = tform2rotm(A_6_1);
rotm_2 = tform2rotm(A_6_2);
rotm_3 = tform2rotm(A_6_3);
rotm_4 = tform2rotm(A_6_4);
rotm_5 = tform2rotm(A_6_5);

% Translation Vector
trvec_0 = tform2trvec(A_6_0);
trvec_1 = tform2trvec(A_6_1);
trvec_2 = tform2trvec(A_6_2);
trvec_3 = tform2trvec(A_6_3);
trvec_4 = tform2trvec(A_6_4);
trvec_5 = tform2trvec(A_6_5);

% Euler Angles
eul_0 = rotm2eul(rotm_0);
eul_1 = rotm2eul(rotm_1);
eul_2 = rotm2eul(rotm_2);
eul_3 = rotm2eul(rotm_3);
eul_4 = rotm2eul(rotm_4);
eul_5 = rotm2eul(rotm_5);

% Axis/Angle
axang_0 = rotm2axang(rotm_0);
axang_1 = rotm2axang(rotm_1);
axang_2 = rotm2axang(rotm_2);
axang_3 = rotm2axang(rotm_3);
axang_4 = rotm2axang(rotm_4);
axang_5 = rotm2axang(rotm_5);