function [cost] = funcion_costo_ty(x, N, u_ref, u_ref_p, u, u_p, q_p,  tau)
                                             
he = [];
for k=1:N
    %% Get Values for the PID
    theta_ref = u_ref(1,k);
    theta_ref_p = u_ref_p(1, k);
    theta = u(1,k);
    theta_p = u_p(1,k);
    ty = tau(2, k);
    
    %% Aproximation
    ty_e =  x(1)*(theta_ref) - x(2)*(theta)  +x(3)*theta_p + x(4)*q_p(k);  

    %% Error Vector
    he = [he; ty_e - ty];
end
cost = norm(he,2);
end