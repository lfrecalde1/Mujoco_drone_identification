function [quaternion] = euler_to_quaternion(euler)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
quaternion = zeros(4,length(euler));
for k = 1:length(euler)
    %% Cherck this before Euler to quaternion
    new_euler = [euler(3, k); euler(2, k); euler(1, k)];
    
   quaternion(:, k) =  eul2quat(new_euler','ZYX');
end
end

