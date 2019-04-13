function [R, T] = relativepose(sCoord2D, tCoord2D, K)
 % build that A matrix
 A = [];
 twos = zeros([8 3]);
 ones = zeros([8 3]);
 % acutally comprose a matrix from a^j columns
 for i=1:8
    two = [sCoord2D(i,:) 1] * K^-1;
    one = [tCoord2D(i,:) 1] * K^-1;
    
    twos(i, :) = two;
    ones(i, :) = one;
    
    x2 = two(1);
    y2 = two(2);
    z2 = two(3);
    
    x1 = one(1);
    y1 = one(2);
    z1 = one(3);
    
    aj = [x2*x1 x2*y1 x2*z1 y2*x1 y2*y1 y2*z1 z2*x1 z2*y1 z2*z1];
    A = [A; aj];
 end
 
 [~, ~, Vt] = svd(A);
 V = Vt';
 
 % this gives us the non-essential E matrix
 e = reshape(V(1:9, 9), [3 3])';
 [u, s, v] = svd(e);
 % this matrix is essential due to the \Sigma haveing diag{1,1,0}
 e_proj = u * [1 0 0; 0 1 0; 0 0 0] * v;
 
 % rotation matrix
 r_z = [0 -1 0; 1 0 0; 0 0 1];
  
 % 4 pose 
 % decompose e projection to use in calculate different pose
 [U, S, Vt] = svd(e_proj);
 
 % option 1
 T_hat_1 = U*r_z*S*U';
 R_1 = U*r_z'*Vt;
 T_1 = [T_hat_1(3,2);T_hat_1(1,3);T_hat_1(2,1)];
 
 % option 2
 T_hat_2 = U*r_z'*S*U';
 R_2 = U*r_z*Vt;
 T_2 = [T_hat_2(3,2);T_hat_2(1,3);T_hat_2(2,1)];
 
 % now calc R T for -E
 [U, S, Vt] = svd(-e_proj);
 
 % option 3
 T_hat_3 = U*r_z*S*U';
 R_3 = U*r_z'*Vt;
 T_3 = [T_hat_3(3,2);T_hat_3(1,3);T_hat_3(2,1)];
 
 % option 4
 T_hat_4 = U*r_z'*S*U';
 R_4 = U*r_z*Vt;
 T_4 = [T_hat_4(3,2);T_hat_4(1,3);T_hat_4(2,1)];
 
 % set up for easy selection
 rs = [R_1; R_2; R_3; R_4];
 ts = [T_1; T_2; T_3; T_4];
 thats = [T_hat_1;T_hat_2;T_hat_3;T_hat_4];
 
 R = [];
 T = [];
 T_hat = [];

 % picked a point to solve the system of equations with
 o = ones(2,:)';
 t = twos(2,:)';
 
 for i=1:4
     % the only option that gives lambda_1 and lambda_2 that are positive
     R_ = rs(3*(i-1) + 1:3*i, :);
     T_ = ts(3*(i-1) + 1:3*i, :);
      
     % solve system of linear equations for lambda
     % create system of equations
     syms lambda_1 lambda_2
     eqn1 = lambda_1*(o'*o) == lambda_2*(o'*R_*t) + o'*T_;
     eqn2 = lambda_1*((R_*t)'*o) == lambda_2*((R_*t)'* R_*t) + (R_*t)'*T_;
        
     % write solution to variables
     sol = solve([eqn1, eqn2], [lambda_1, lambda_2]);
     lambda_1 = double(sol.lambda_1);
     lambda_2 = double(sol.lambda_2);
     
     % save solutions of positive depth constraint is satisfied
     if lambda_1 > 0 && lambda_2 > 0
        R = R_;
        T = T_;
        T_hat = thats(3*(i-1) + 1:3*i, :); % used for calculating E
     end
 end

end