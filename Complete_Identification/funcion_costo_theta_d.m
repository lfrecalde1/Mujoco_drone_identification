function [cost] = funcion_costo_theta_d(x, N, u_ref, u, u_p, theta_d)
                                             
he = [];
for k=1:N
    %% Get Values for the PID
    ul_ref = u_ref(1,k);
    ul = u(1,k);
    ul_p = u_p(1,k);
    theta = theta_d(1, k);
    
    %% Aproximation
    ty_e =  x(1)*(ul_ref) - x(2)*(ul)  +x(3)*ul_p;  

    %% Error Vector
    he = [he; ty_e - theta];
end
cost = norm(he,2);
end