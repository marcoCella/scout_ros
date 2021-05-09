
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

% /gt_pose data
gt_quaternion = [0.973713457584 0.0204658899456 0.00186488381587 -0.22684790194];
gt_q = quaternion(gt_quaternion);
gt_q_inv = quaternion([0.982173144817 -0.0190145038068 0.00571734644473 0.186926841736]); % inverse of quaternion
gt_pose = [-0.420354783535 0.289032250643 0.331286072731]';

% /scout_odom data    position: 
sc_quaternion = [0.589118022045 0.0 0.0 -0.808047001171];
sc_q = quaternion(sc_quaternion);
sc_q_inv = quaternion([0.64884654485 -0.0 -0.0 0.760919286939]); % inverse of quaternion
sc_pose = [0.279158768242 -0.163530515374 0.0]';


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




