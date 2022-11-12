function I = matrix_invert(X)
  [m n] = size(X);
  for i = 1:m
      for j = 1:n
          if i ~= j
              X(i, j) = -X(i, j);
          end
      end
  end
end
