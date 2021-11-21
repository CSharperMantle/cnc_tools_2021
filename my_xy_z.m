% my_xy_z.m: Assignment for 2021-11-20.
% 
% Modeling for an XYZ set with X & Y on the workpiece
% and Z on cutting machineries.
% 
% Author: Rong Bao

clear variables;

syms x y z;  % X, Y, Z axes
syms ErrLin_xX ErrLin_yX ErrLin_zX;  % Linear error on X
syms ErrRot_xX ErrRot_yX ErrRot_zX;  % Rotation error on X
syms ErrLin_xY ErrLin_yY ErrLin_zY;  % Linear error on Y
syms ErrRot_xY ErrRot_yY ErrRot_zY;  % Rotation error on Y
syms ErrLin_xZ ErrLin_yZ ErrLin_zZ;  % Linear error on Z
syms ErrRot_xZ ErrRot_yZ ErrRot_zZ;  % Rotation error on X

List_Err_X = [ ErrLin_xX ErrLin_yX ErrLin_zX ErrRot_xX ErrRot_yX ErrRot_zX ];
List_Err_Y = [ ErrLin_xY ErrLin_yY ErrLin_zY ErrRot_xY ErrRot_yY ErrRot_zY ];
List_Err_Z = [ ErrLin_xZ ErrLin_yZ ErrLin_zZ ErrRot_xZ ErrRot_yZ ErrRot_zZ ];

Mat_Trans_RY = [1 0 0 0;
                0 1 0 y;
                0 0 1 0;
                0 0 0 1] * create_E5(List_Err_Y, 1);
Mat_Trans_YX = [1 0 0 x;
                0 1 0 0;
                0 0 1 0;
                0 0 0 1] * create_E5(List_Err_X, 1);
Mat_Trans_XW = eye(4);

Mat_Trans_RW = Mat_Trans_RY * Mat_Trans_YX * Mat_Trans_XW;
Mat_Trans_WR = inv(Mat_Trans_XW) * inv(Mat_Trans_YX) * inv(Mat_Trans_RY);

Mat_Trans_RZ = [1 0 0 0;
                0 1 0 0;
                0 0 1 z;
                0 0 0 1] * create_E5(List_Err_Z, 1);
Mat_Trans_ZT = eye(4);

Mat_Trans_RT = Mat_Trans_RZ * Mat_Trans_ZT;

Mat_Trans_WT = Mat_Trans_WR * Mat_Trans_RT;

Mat_Trans_WT_Ideal = [1 0 0 -x;
                      0 1 0 -y;
                      0 0 1 z;
                      0 0 0 1];

% Mat_Error_WT = inv(Mat_Trans_WT_Ideal) * Mat_Trans_WT;  % Slower
Mat_Error_WT = Mat_Trans_WT_Ideal \ Mat_Trans_WT;
Err_X = expand(Mat_Error_WT(1, 4));
Err_Y = expand(Mat_Error_WT(2, 4));
Err_Z = expand(Mat_Error_WT(3, 4));
