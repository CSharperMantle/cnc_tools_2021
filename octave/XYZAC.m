% RRTTT-type CNC Tools Error Model

clear variables;

syms x y z a c; % Translation & rotation
syms Er_xX Er_yX Er_zX; % ERot over X
syms E_xX E_yX E_zX; % ETrans over X
syms Er_xY Er_yY Er_zY; % ERot over Y
syms E_xY E_yY E_zY; % ETrans over Y
syms Er_xZ Er_yZ Er_zZ; % ERot over Z
syms E_xZ E_yZ E_zZ;% ETrans over Z
syms S_xy S_xz S_yz; % Vertical

syms Er_xA Er_yA Er_zA; % PDGEs over A
syms E_xA E_yA E_zA;
syms Er_aAy Er_bAy Er_cAy; % PIGEs over A
syms E_xAy E_yAy E_zAy;
syms Er_xC Er_yC Er_zC; % PDGEs over C
syms E_xC E_yC E_zC;
syms Er_bCA; % PIGEs over C
syms E_yCA;

E_X = [E_xX E_yX E_zX Er_xX Er_yX Er_zX]; % Errors over X
E_Y = [E_xY E_yY E_zY Er_xY Er_yY Er_zY]; % Errors over Y
E_Z = [E_xZ E_yZ E_zZ Er_xZ Er_yZ Er_zZ]; % Errors over Z
E_a = [[E_xA E_yA E_zA Er_xA Er_yA Er_zA];
       [E_xAy E_yAy E_zAy Er_aAy Er_bAy Er_cAy]]; % Errors over A
E_c = [[E_xC E_yC E_zC Er_xC Er_yC Er_zC];
       [0 E_yCA 0 0 Er_bCA 0]]; % Errors over C

% Workpiece transformation chain
T_RY = [
         [1 -S_xy 0 0];
         [S_xy 1 0 0];
         [0 0 1 0];
         [0 0 0 1]
       ] * [
         [1 0 0 0];
         [0 1 0 y];
         [0 0 1 0];
         [0 0 0 1]
       ] * create_E5(E_Y, 1);
T_YR = matrix_invert(T_RY);

T_YX = [
         [1 0 0 x];
         [0 1 0 0];
         [0 0 1 0];
         [0 0 0 1]
       ] * create_E5(E_X, 1);
T_XY = matrix_invert(T_YX);

T_XA = expand(create_E5(E_a, 2) * create_E5(E_a, 1) * Rotate_X(a));
T_AX = matrix_invert44(T_XA);

T_AC = expand(create_E5(E_c, 2) * create_E5(E_c, 1) * Rotate_Z(c));
T_CA = matrix_invert44(T_AC);

T_CW = eye(4);

T_WR = expand(T_CA * T_AX * T_XY * T_YR); % W-C-A-X-Y-R

% Milling head transformation chain
T_RZ = [
         [1 0 S_xz 0];
         [0 1 -S_yz 0];
         [-S_xz S_yz 1 0];
         [0 0 0 1]
       ] * [
         [1 0 0 0];
         [0 1 0 0];
         [0 0 1 z];
         [0 0 0 1]
       ] * create_E5(E_Z, 1);
T_Zt = eye(4);
T_Rt = T_RZ * T_Zt; % R-Z-t

T_Wt = expand(T_WR * T_Rt)
