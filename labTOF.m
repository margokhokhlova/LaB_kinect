%% lab TOF
%% Images from TOF and Color Camera
clear all; close all; clc; 
cd ('tof_ccd_images_for_colored_3D_reconstruction');
for imgNum = 1:3
    colImg = imread(strcat('col_img_', num2str(imgNum),'.jpg'));
    subplot(2,3,imgNum); imshow(colImg); title('Color Image');
end

for imgNum = 1:3
    load(strcat('dep_img_', num2str(imgNum),'.mat'));
    subplot(2,3,imgNum+3); imshow(depImg, []); title('Depth Image');
end
[H, W, ~] = size(depImg); %take the size of the Depth Image
%remove the negative depth values if there are any -  these are errors

% TO DO 
% store the result in DepImg;

%% 3D Reconstruction
%Back project the depth data to the 3D space to obtain 3D coordinates for every single pixel.
% Depth camera parameters
% You are given all the camera parameters:
fx_d = 245.79171;
fy_d = 249.67334;
cx_d = 122.71519;
cy_d = 71.98096;
% Color camera parameters
fx_rgb = 2095.51991;
fy_rgb = 2055.63143;
cx_rgb = 1188.99933;
cy_rgb = 623.16576;
% Rotation matrix
R = inv([ 0.9999 -0.0044 -0.0126; 0.0044 1.0000 0.0014; 0.0126 -0.0015 0.9999]);
% Translation vector
T = -[ 0.06764654, -1.4391632009665779e-04, -7.9356552371601212e-03 ]';

% Compute the world coordinates using only the fx, fy, cx, cy
% I am giving you the formula here, you should just substitute Z with the
% correct value of the depth image.

[xx, yy] = meshgrid(1:W,1:H); % xx, yy contains actual image coordinates (x,y)
for i=1:H
    for j=1:W
%TO DO      
     X(i,j)% = Z * (xx(i,j)-cx_d) / fx_d;  
     Y(i,j)%= = Z* (yy(i,j)-cy_d ) / fy_d ;     
        

    end
end
% Generate colored 3D point cloud as a part of 3D reconstruction
% you will need to use R and T to get XYZ coordinates 
% Reminder:
% XYZ =R* (X1, Y1, Z1) + T;
% Caclulate XYZ!
for i=1:H
       for j=1:W         
         
% TO DO           
   % XYZ = 
   XD(i,j)=XYZ(1);
   YD(i,j)=XYZ(2);
   ZD(i,j)=XYZ(3);
     
  
           
       end
end
% (b) project into image coordinate
% Project into rgb coordinate frame
[W1,H1]=size(colImg(:,:,3)); %size for color image


for i=1:H
    for j=1:W
     xrgb(i,j)=XD(i,j)*fx_rgb/ZD(i,j)+cx_rgb ;  
     yrgb(i,j)=YD(i,j)*fx_rgb/ZD(i,j)+cy_rgb ;
     %Remnove some outliers 
     if  xrgb(i,j)<1
         xrgb(i,j)=1;
     elseif xrgb(i,j)>W1
         xrgb(i,j)=W1;
     else  xrgb(i,j)= xrgb(i,j);
     end
     
       if  yrgb(i,j)<1
         yrgb(i,j)=1;
     elseif yrgb(i,j)>H1
         yrgb(i,j)=H1;
     else  yrgb(i,j)= yrgb(i,j);
     end
   

    end
end

for i=1:H
    for j=1:W
        imgRgb(i,j, :)=colImg(round(xrgb(i,j)),round(yrgb(i,j)),:);
    end
end
% (d) Construct the projected image and show it
% take not all the pixels from color image, but with a step to
% have a corresponding points for Depth Image with lower resolution


% Generatea colored point cloud of the scene. 
% You now have all the data to plot a 3d POINT CLOUD:
% imgRgb, DepImg(our Z!) and X and Y.
% use scatter3 command
% don't forget to add the color!!!

% TO DO
