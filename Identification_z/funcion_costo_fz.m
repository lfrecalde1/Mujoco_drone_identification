function [cost] = funcion_costo_fz(x, N, u_ref, u, u_p,  F)
                                             
he = [];
for k=1:N
    %% Get Values for the PID
    un_ref = u_ref(3,k);
    un = u(3,k);
    un_p = u_p(3,k);
    fz = F(3, k);
    
    %% Aproximation
    u_T =  x(1)*9.8 + x(2)*un_ref - x(3) * un + x(4)*un_p;

    %% Error Vector
    he = [he; u_T - fz];
end
cost = norm(he,2);

end