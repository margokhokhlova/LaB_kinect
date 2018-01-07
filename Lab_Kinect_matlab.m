% Practical work with the data obtained with the kinect v 2 device

% In this lab you will learn some basics about depth image processing and usage of depth data in 
% Computer Vision. This is the main file which will guide you through this
% TP. Some functions might need a Computer Vision Toolbox to run. If they
% do not work, just comment them and perform the part which doesn't require
% any special toolbox.


% all the parts that should be filled in are marked by 'TO DO'
%%
% Kinect sensor provides with 3 images, infra-red, color and depth image.
% Here we will work with depth and rgb images. Let's first load them:

% read depth image
% read color image
I_1 = imread('001589_color.png');
D_1=imread('001589_depth.png');

%%
% imshow the initial images
figure
subplot(1,2,1), imshow(I_1); title('Color');
subplot(1,2,2), imshow(D_1, []); title('Depth');




%% The depth image filtering
% Open the matrix with depth values (D_1) and take a look at the data. 
% The depth provided is the real depth for each point on the 2d image in
% mm.
% write a function to filter out the depth values with a given threshold
% ex filtered_depth = filterValues(D_1, threshold);
% imshow(filtered_depth);
%  TO DO

%% show point cloud (Computer Vision System toolbox is needed)
% Knowing the intrinsic parameters of a camera, we can always find real
% scene coordinates (X,Y,Z) using a depth image. 
% Here the calibration is performed beforehand, and the values of F_x, cx,
% cy are
% Please read more on camera calibration if you are interested
% Tsai, R. (1987). A versatile camera calibration technique for high-accuracy 
% 3D machine vision metrology using off-the-shelf TV cameras and lenses. 
% IEEE Journal on Robotics and Automation, 3(4), 323-344.
pc=zeros([size(D) 3]);
W=size(D,2);
H=size(D,1);
f=3.66; % focal length of the camera
for indWidth = 1:W
    for indHeight= 1:H
        % copy z value
        pc(indHeight,indWidth,3)=D(indHeight,indWidth);
        % calc x value
        pc(indHeight,indWidth,1)=-(pc(indHeight,indWidth,3)/f)*...
            ((indWidth-W/2)+0.0256);
        % calc y value
        pc(indHeight,indWidth,2)=-(pc(indHeight,indWidth,3)/f)*...
            ((indHeight-H/2)+0.022);
    end
end
X=pc(:,:,1);
% X=X(:);
Y=pc(:,:,2);
% Y=Y(:);
Z=-pc(:,:,3);
Z(Z==0)=NaN;

Surface=surf(X,Y,Z,'edgecolor','none','facecolor','interp');
lighting gouraud
camlight
% colormap(repmat(winter,20,1))
axis image
axis vis3d
xlabel('X axis')
ylabel('Y axis')
zlabel('Z axis')
%% or plot with color 
% Surface=surf(X,Y,Z,  I_1);


%% And visualize your filtered image here
% TO DO



%% adding a mask of  a user
% Kinect SDK also provides with a human detection features
% Lets load a binary mask of a user provided by kinect and stored

% Now let's remove all the depth data which do not correspond to the user
% from the depth image
% hint zero out all the depth values using the mask (element-wise matrix
% multiplication can be a good idea)

% TO DO

% store the resulting depth map in USER_1

%% PART 2. Skeletons (joints)

%% working with skeletons
% load data
% open the txt file with joints data and check how the data are organized
% there are 25 joints estimated by the kinect v2, each joint has 3
% Coordinates X, Y, Z, 4 coordinates for the orientation and one value
% which corresponds to the nature of the joint estimation  (more on this
% data here:  


% transform data and plot
%% working with simple 3D-2D projcections 
% first joint frame corresponds to our color image 
% the joints are in a real world coordinates (3D, X, Y, Z for each joint)
% if we the intrinsic parameters of our camera, we can perform a 3D-2D
% transformation of any point by a following perspective projection:
% Basic Perspective Projection
% x=1/sy *f*X/Z +ox
% y =1/sx * f * Y/Z +oy
% more on it 'https://en.wikipedia.org/wiki/3D_projection'


% Now project the joints from the first frame to the color image  (3D-2D)


% How to check: The projected joints are already on the color image -
% compare with the coordinates obtained 


%% Find all the data corresponding to the right ankle joint evaluation in time
% There are lots of interesting applications where joints can be used.
% Today we will try a simple one  - we will estimate the frequency of a
% persons walk using the joint data.
% info you will need here: fps of the Kinect v 2 camera. We will take this
% as 30 frames per second
% gait frequency - number of gait cycles in a time span (here let's take it
% as 30 seconds)
% One gait cycle is 
% TO DO


