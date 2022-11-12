clc
syms x y z d;
syms Er_xX Er_yX Er_zX Er_FxX d; % X轴旋转误差
syms E_xX E_yX E_zX ; % X轴线性误差
syms Er_xY Er_yY Er_zY d; % Y轴旋转误差
syms E_xY E_yY E_zY d; % Y轴线性误差
syms Er_xZ Er_yZ Er_zZ d; % Z轴旋转误差
syms E_xZ E_yZ E_zZ d;% Z轴线性误差
syms S_xy S_xz S_yz d; % 三轴之间的垂直度误差

E=[x y z]; % 用于存放机床三轴的位移
E_X=[E_xX E_yX E_zX Er_xX Er_yX Er_zX]; % 用于存放X轴的所有误差
E_Y=[E_xY E_yY E_zY Er_xY Er_yY Er_zY]; % 用于存放Y轴的所有误差
E_Z=[E_xZ E_yZ E_zZ Er_xZ Er_yZ Er_zZ]; % 用于存放Z轴的所有误差

% 计算有误差情况下的运动矩阵
T_RY=[1 -S_xy 0 0;S_xy 1 0 0;0 0 1 0;0 0 0 1]*[1 0 0 0;0 1 0 y;0 0 1 0;0 0 0 1]*create_E5(E_Y,1);
T_YX=[1 0 0 x;0 1 0 0;0 0 1 0;0 0 0 1]*create_E5(E_X,1);
T_XW=eye(4);
T_RW_a=T_RY*T_YX*T_XW; % R-Y-X-W 工件链；
T_WR_a=matrix_invert44(T_YX)*matrix_invert44(T_RY); % 对T_RW_a取逆

T_RZ=[1 0 S_xz 0;0 1 -S_yz 0;-S_xz S_yz 1 0;0 0 0 1]*[1 0 0 0;0 1 0 0;0 0 1 z;0 0 0 1]*create_E5(E_Z,1);
T_Zt=eye(4);
T_Rt_a=T_RZ*T_Zt; % R-Z-t 刀具链；

T_Wt_a=T_WR_a*T_Rt_a;

% 无误差情况下的理想位置矩阵
T_Wt_i=[1 0 0 -x;0 1 0 -y;0 0 1 z;0 0 0 1];

% 计算误差运动矩阵
Te_Wt_a=inv(T_Wt_i)*T_Wt_a; 

X_e=expand(Te_Wt_a(1,4))
Y_e=expand(Te_Wt_a(2,4))
Z_e=expand(Te_Wt_a(3,4))
