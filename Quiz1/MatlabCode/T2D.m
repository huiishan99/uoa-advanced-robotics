function [T] = T2D(x, y, q)
%T2D returns 2D homogeneous trannsformation matrix 
%   from a target frame to a world frame
%   Input
%   - x: x coordinate of target x-origin in a world frame
%   - y: y coordinate of target y-origin in a world frame
%   - q: angle from a world to target frame
%   Output
%   - T: 3*3 matrix
T = [
    cos(q), -sin(q), x;
    sin(q), cos(q), y;
    0, 0, 1
    ];
end
%       Author: Keitaro Naruse
%       Date: 2022-04-12
%       MIT License