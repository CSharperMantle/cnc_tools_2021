 % RRTTT型五轴加工中心 几何误差元素数学模型
clear all
clc
syms x y z a c d; % 机床位移和转角
syms Er_xX Er_yX Er_zX d; % X轴旋转误差
syms E_xX E_yX E_zX d; % X轴线性误差
syms Er_xY Er_yY Er_zY d; % Y轴旋转误差
syms E_xY E_yY E_zY ; % Y轴线性误差
syms Er_xZ Er_yZ Er_zZ d; % Z轴旋转误差
syms E_xZ E_yZ E_zZ d;% Z轴线性误差
syms S_xy S_xz S_yz d; % 三轴之间的垂直度误差

syms Er_xA Er_yA Er_zA d;% A轴PDGEs误差
syms E_xA E_yA E_zA d;
syms Er_aAy Er_bAy Er_cAy d; % A轴PIGEs误差
syms E_xAy E_yAy E_zAy d;
syms Er_xC Er_yC Er_zC d;% C轴PDGEs误差
syms E_xC E_yC E_zC d;
syms Er_bCA d; % C轴PIGEs误差
syms E_yCA d;

E_X=[E_xX E_yX E_zX Er_xX Er_yX Er_zX]; % 用于存放X轴的所有误差
E_Y=[E_xY E_yY E_zY Er_xY Er_yY Er_zY]; % 用于存放Y轴的所有误差
E_Z=[E_xZ E_yZ E_zZ Er_xZ Er_yZ Er_zZ]; % 用于存放Z轴的所有误差
E_a=[E_xA E_yA E_zA Er_xA Er_yA Er_zA; % 用于存放A轴的所有误差
    E_xAy E_yAy E_zAy Er_aAy Er_bAy Er_cAy];
E_c=[E_xC E_yC E_zC Er_xC Er_yC Er_zC; % 用于存放C轴的所有误差
    0 E_yCA 0 0 Er_bCA 0 ];

% 工件链
T_RY=[1 -S_xy 0 0;S_xy 1 0 0;0 0 1 0;0 0 0 1]*[1 0 0 0;0 1 0 y;0 0 1 0;0 0 0 1]*create_E5(E_Y,1);
T_YR=matrix_invert(T_RY);

T_YX=[1 0 0 x;0 1 0 0;0 0 1 0;0 0 0 1]*create_E5(E_X,1);
T_XY=matrix_invert(T_YX);

T_XA=expand(create_E5(E_a,2)*create_E5(E_a,1)*Rotoate_X(a));
T_AX=matrix_invert44(T_XA);

T_AC=expand(create_E5(E_c,2)*create_E5(E_c,1)*Rotoate_Z(c));
T_CA=matrix_invert44(T_AC);

T_CW=eye(4);

%T_RW=T_RY*T_YX*T_XA*T_AC*T_CW; % R-Y-X-A-C-W 工件链
T_WR=expand(T_CA*T_AX*T_XY*T_YR); % W-C-A-X-Y-R 工件链

% 刀具链
T_RZ=[1 0 S_xz 0;0 1 -S_yz 0;-S_xz S_yz 1 0;0 0 0 1]*[1 0 0 0; 0 1 0 0;0 0 1 z; 0 0 0 1]*create_E5(E_Z,1);
T_Zt=eye(4);
T_Rt=T_RZ*T_Zt; % R-Z-t 刀具链

T_Wt=expand(T_WR*T_Rt)