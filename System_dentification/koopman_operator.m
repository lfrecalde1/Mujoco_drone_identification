function [Gamma] = koopman_operator(X)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
%% Definition of the koopman vector
Gamma =zeros (12, 1);

%% Definition of the first 6 elements
Gamma(1:6) = X(1:6);

%% DEfinition of hte other elements using knowladge
Gamma(7) = X(1)*X(2);
Gamma(8) = X(1)*X(3);
Gamma(9) = X(2)*X(3);

Gamma(10) = X(4)*X(5);
Gamma(11) = X(4)*X(6);
Gamma(12) = X(5)*X(6);

end

