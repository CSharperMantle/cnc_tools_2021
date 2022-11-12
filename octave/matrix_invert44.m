function I_4 = matrix_invert44(X)
  a = X(1:3,1:3);
  b = X(1:3,4);
  a_new = a';
  b_new = -a'*b;
  I_4 = [[a_new b_new]; [0 0 0 1]];
end
