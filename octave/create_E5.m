function P = create_E5(Y, i)
  P = [
       [1 -Y(i, 6) Y(i, 5) Y(i, 1)];
       [Y(i, 6) 1 -Y(i, 4) Y(i, 2)];
       [-Y(i, 5) Y(i, 4) 1 Y(i, 3)];
       [0 0 0 1]
      ];
end
