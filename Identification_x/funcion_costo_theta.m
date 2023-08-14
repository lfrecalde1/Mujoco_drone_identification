function [cost] = funcion_costo_theta(x, N, u_ref, u, u_p, u_pp,  tau)
      
chi = [ 0.0006    0.0012   -0.0018    0.0009];
he = [];
for k=1:N
    %% Get Values for the PID
    theta_ref = u_ref(2,k);
    theta = u(2,k);
    theta_p = u_p(2,k);
    theta_pp = u_pp(2, k);
    ty = tau(2, k);
    
    phi_p = u_p(1, k);
    psi_p = u_p(3, k);
    
    %% Aproximation
    ty_e =  chi(1)*(theta_ref) - chi(2)*(theta)  + chi(3)*theta_p + chi(4)*theta_pp;
    %theta_pp_estimate = (-x(1)/x(2))*phi_p*psi_p - (x(2)/x(2))*phi_p*psi_p + (x(3)/x(2))*phi_p*psi_p + (1/x(2))*ty_e;
    theta_pp_estimate =  (-x(1)/x(2))*(1/(x(3)/x(4)))*phi_p*psi_p - (x(5)/x(6))*(1/(x(7)/x(8)))*phi_p*psi_p + (x(9)/x(10))*(1/(x(11)/x(12)))*phi_p*psi_p + (1/x(13))*(x(14)*(theta_ref) - x(15)*(theta)  + x(16)*theta_p);

    %% Error Vector
    he = [he; theta_pp_estimate - theta_pp];
end
cost = norm(he,2);
end