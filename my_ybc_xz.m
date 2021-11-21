% my_ybc_xz.m: Assignment for 2021-11-20.
% 
% Modeling for an BCXYZ set with Y, B & C on
% the workpiece and X & Z on cutting machineries.
% 
% Author: Rong Bao

clear variables;

syms x y z b c;
syms ErrLin_xX ErrLin_yX ErrLin_zX;  % Linear error on X
syms ErrRot_xX ErrRot_yX ErrRot_zX;  % Rotation error on X
syms ErrLin_xY ErrLin_yY ErrLin_zY;  % Linear error on Y
syms ErrRot_xY ErrRot_yY ErrRot_zY;  % Rotation error on Y
syms ErrLin_xZ ErrLin_yZ ErrLin_zZ;  % Linear error on Z
syms ErrRot_xZ ErrRot_yZ ErrRot_zZ;  % Rotation error on X

syms ErrRot_xB ErrRot_yB ErrRot_zB;  % PDGEs on B
syms ErrLin_xB ErrLin_yB ErrLin_zB;
syms ErrRot_aBy ErrRot_bBy ErrRot_cBy;  % PIGEs on B
syms ErrLin_xBy ErrLin_yBy ErrLin_zBy;
syms ErrRot_xC ErrRot_yC ErrRot_zC;  % PDGEs on C
syms ErrLin_xC ErrLin_yC ErrLin_zC;
syms ErrRot_bCA;  % PIGEs on C
syms ErrLin_yCA;

List_Err_X = [ ErrLin_xX ErrLin_yX ErrLin_zX ErrRot_xX ErrRot_yX ErrRot_zX ];
List_Err_Y = [ ErrLin_xY ErrLin_yY ErrLin_zY ErrRot_xY ErrRot_yY ErrRot_zY ];
List_Err_Z = [ErrLin_xZ ErrLin_yZ ErrLin_zZ ErrRot_xZ ErrRot_yZ ErrRot_zZ];
List_Err_B = [ErrLin_xB ErrLin_yB ErrLin_zB ErrRot_xB ErrRot_yB ErrRot_zB;
    ErrLin_xBy ErrLin_yBy ErrLin_zBy ErrRot_aBy ErrRot_bBy ErrRot_cBy];
List_Err_C = [ErrLin_xC ErrLin_yC ErrLin_zC ErrRot_xC ErrRot_yC ErrRot_zC;
    0 ErrLin_yCA 0 0 ErrRot_bCA 0 ];

% Workpiece chain
Mat_Trans_RY = [1 0 0 0;
                0 1 0 y;
                0 0 1 0;
                0 0 0 1] * create_E5(List_Err_Y, 1);
Mat_Trans_YB = expand(create_E5(List_Err_B, 2) ...
    * create_E5(List_Err_B, 1) ...
    * create_rotate_X(b));
Mat_Trans_BC = expand(create_E5(List_Err_C, 2) ...
    * create_E5(List_Err_C, 1) ...
    * create_rotate_Z(c));
Mat_Trans_CW = eye(4);

Mat_Trans_WR = inv(Mat_Trans_RY) ...
    * inv(Mat_Trans_YB) ...
    * inv(Mat_Trans_BC) ...
    * inv(Mat_Trans_CW);

Mat_Trans_RZ = [1 0 0 0;
                0 1 0 0;
                0 0 1 z;
                0 0 0 1] * create_E5(List_Err_Z, 1);
Mat_Trans_ZT = eye(4);

Mat_Trans_RT = Mat_Trans_RZ * Mat_Trans_ZT;

Mat_Trans_WT = Mat_Trans_WR * Mat_Trans_RT;

Err_WT = expand(Mat_Trans_WT);
