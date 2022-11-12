function R_Z = Rotate_Z(c)
  R_Z = [
        [cos(c) -sin(c) 0 0];
        [sin(c) cos(c) 0 0];
        [0 0 1 0];
        [0 0 0 1]
      ];
end
