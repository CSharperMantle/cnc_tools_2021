function X_ni=matrix_invert44(X)
a=X(1:3,1:3);
b=X(1:3,4);
a_new=a';
b_new=-a'*b;
X_ni=[a_new b_new];
X_ni=[X_ni;0 0 0 1];