
%% This script computes the rotation and translation from/to odom to/from world (static frames)

% we copied 2 messages with rostopic echo from /scout_odom and /gt_pose
% /gt_pose is not accurate in the first seconds, so we waited for some
% seconds and as soon as we saw a coherent /gt_pose we copied its pose and
% the corresponding /scout_odom pose.
% We assume that at the beginning /scout_odom is correct, so we can use its
% pose as reference to find the rototranslation from odom (frame_id of
% /scout_odom) and world (frame id of /gt_pose)

clearvars
clc

%{
% /gt_pose data
gt_quaternion = [0.982173144817 0.0190145038068 -0.00571734644473 -0.186926841736];
gt_q = quaternion(gt_quaternion);
gt_q_inv = quaternion([gt_quaternion(1) -gt_quaternion(2) -gt_quaternion(3) -gt_quaternion(4)]);  % inverse of quaternion
gt_pose = [ -0.123131193221 0.0424058549106 0.341845780611]';

% /scout_odom data    position: 
sc_quaternion = [0.653461393927 0.0 0.0 -0.756959844805];
sc_q = quaternion(sc_quaternion);
sc_q_inv = quaternion([sc_quaternion(1) -sc_quaternion(2) -sc_quaternion(3) -sc_quaternion(4)]); % inverse of quaternion
sc_pose = [0.191617150328 -0.544396529413 0.0]';
%}

%{
% /gt_pose data
gt_quaternion = [0.987053155899 0.0218208972365 0.00395742757246 -0.158853128552];
gt_q = quaternion(gt_quaternion);
gt_q_inv = quaternion([gt_quaternion(1) -gt_quaternion(2) -gt_quaternion(3) -gt_quaternion(4)]);  % inverse of quaternion
gt_pose = [ 0.0508471392095 -0.0663025602698 0.340004742146]';

% /scout_odom data    position:   position: 
sc_quaternion = [0.67781544492 0.0 0.0 -0.735232087594];
sc_q = quaternion(sc_quaternion);
sc_q_inv = quaternion([sc_quaternion(1) -sc_quaternion(2) -sc_quaternion(3) -sc_quaternion(4)]); % inverse of quaternion
sc_pose = [0.178667966593 -0.654373882761 0.0]';
%}

%{
% /gt_pose data
gt_quaternion = [0.987590551376 0.0214921869338 0.00367050012574 -0.15553034842];
gt_q = quaternion(gt_quaternion);
gt_q_inv = quaternion([gt_quaternion(1) -gt_quaternion(2) -gt_quaternion(3) -gt_quaternion(4)]); % inverse of quaternion
gt_pose = [ 0.0598290190101 -0.0706361308694 0.339890509844]';

% /scout_odom data    position: 
sc_quaternion = [0.704669483885 0.0 0.0 -0.709535706277];
sc_q = quaternion(sc_quaternion);
sc_q_inv = quaternion([sc_quaternion(1) -sc_quaternion(2) -sc_quaternion(3) -sc_quaternion(4)]); % inverse of quaternion
sc_pose = [0.173034957863 -0.780878450376 0.0]';
%}

%
% /gt_pose data 
%gt_quaternion = [0.975580275059 0.0191441625357 0.000759054382797 -0.218805849552];
gt_quaternion = [ 0.941202027977371   0.018904903487771   0.003111232224275  -0.337300559426192];
gt_q = quaternion(gt_quaternion);
gt_q_inv = quaternion([gt_quaternion(1) -gt_quaternion(2) -gt_quaternion(3) -gt_quaternion(4)]); % inverse of quaternion
gt_pose = [-0.420354783535 0.289032250643 0.331286072731]';

% /scout_odom data    position: 
sc_quaternion = [0.596044497658 0.0 0.0 -0.802951403767];
sc_q = quaternion(sc_quaternion);
sc_q_inv = quaternion([sc_quaternion(1) -sc_quaternion(2) -sc_quaternion(3) -sc_quaternion(4)]); % inverse of quaternion
sc_pose = [0.323461636181 -0.0243021184424 0.0]';
%

%% from odom to world
% rotation
od2wo_q_rotation = sc_q*gt_q_inv;
od2wo_mat_rotation= quat2rotm(od2wo_q_rotation);

% translation
od2wo_transl = sc_pose - od2wo_mat_rotation*gt_pose;


%% from world to odom
% rotation
wo2od_q_rotation = gt_q*sc_q_inv;
wo2od_mat_rotation= quat2rotm(wo2od_q_rotation);

% translation
wo2od_transl = -wo2od_mat_rotation*sc_pose + gt_pose;

%% printf
fprintf(1, "Rotation angle: %f", 180/pi*asin(wo2od_mat_rotation(1,2)));

fprintf(1, "\n\n");

[a1, b1, c1, d1] = parts(od2wo_q_rotation);
fprintf(1,"<node pkg=""tf"" type=""static_transform_publisher"" name=""odom2world"" args=""%.9f %.9f %.9f %.9f %.9f %.9f %.9f  odom world 100"" />\n", od2wo_transl(1), od2wo_transl(2), od2wo_transl(3), b1,c1,d1, a1);
[a2, b2, c2, d2] = parts(wo2od_q_rotation);
fprintf(1,"<node pkg=""tf"" type=""static_transform_publisher"" name=""world2odom"" args=""%.9f %.9f %.9f %.9f %.9f %.9f %.9f  world odom 100"" />\n", wo2od_transl(1), wo2od_transl(2), wo2od_transl(3), b2,c2,d2, a2);

fprintf(1, "\n\n");


fprintf("#define GT_X_OFFSET %.9f\n", wo2od_transl(1));
fprintf("#define GT_Y_OFFSET %.9f \n", wo2od_transl(2));
fprintf("#define GT_Z_OFFSET %.9f \n", wo2od_transl(3));
fprintf("#define GT_X_Q_ROT  %.9f \n", b1);
fprintf("#define GT_Y_Q_ROT  %.9f \n", c1);
fprintf("#define GT_Z_Q_ROT  %.9f \n", d1);
fprintf("#define GT_W_Q_ROT  %.9f \n", a1);

fprintf(1, "\n\n");



