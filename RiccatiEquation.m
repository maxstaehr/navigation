C = eye(3);
Q = 1.21*eye(3);
A = [0 0 0;
       0 0 0;
       0 0 0];
B = [0 0 0;
    0 0 0;
    0 0 0];
R = [0.64 0 0;
       0 0.64 0;
       0 0 0.64];
   
   syms S
   res = solve(A*S + S*A' - S*C'*inv(R)*C'*S + Q ==0 );
   Sres = 0.88*eye(3);
   sl = A*Sres + Sres*A' - Sres*C'*inv(R)*C'*Sres + Q