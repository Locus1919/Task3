clear; close all; clc;
edge = imread('edge1.jpg');
edgegray = rgb2gray(edge);
edgedg = im2double(edgegray);
EdgeT = KDTreeSearcher(edgedg);
threededge = pcread('E:/lidar/mav0/result/edges-50¡¢0.05.ply');
twodedge = ones(326484,3)*nan;
twodedge(2,:) = [1,2,3];
t_p2c = [0.2;0;0];
R_p2c = rpy2cv(pi/2,pi/2,-pi/2);
for i = 1 : length(threededge.Location)
    PinL = threededge.Location(i,:).';
    R_p2c = inv(R_p2c);
    PinC = R_p2c*(PinL-t_p2c);
    K = [300,0,300;
    0,300,400;
    0,0,1];
    twodedge(i,:) = K*PinC;
end
