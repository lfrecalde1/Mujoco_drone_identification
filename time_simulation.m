function [t] = time_simulation(v, ts)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
t(1) = 0;
for k=1:length(v)-1
    t(k+1) = t(k) + ts;
end
end

