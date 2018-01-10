clear all; close all; clc; 

for imgNum = 1:3
    colImg = imread(strcat('col_img_', num2str(imgNum),'.jpg'));
    subplot(2,3,imgNum); imshow(colImg);
end

for imgNum = 1:3
    load(strcat('dep_img_', num2str(imgNum),'.mat'));
    subplot(2,3,imgNum+3); imshow(depImg, []);
end