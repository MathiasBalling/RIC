% State [x;y;dx;dy]
% Output [ddx;ddy]
T = 0.1
A = [1 0 T 0; 0 1 0 T; 0 0 1 0; 0 0 0 1]
B = [1/2*T^2 0;0 1/2*T^2; T 0; 0 T]
C = [1 0 0 0; 0 1 0 0]
D = 0;
