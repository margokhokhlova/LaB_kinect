% Practical work with the data obtained with the kinect v 2 device

% In this lab you will learn some basics about depth image processing and usage of depth data in 
% Computer Vision. This is the main file which will guide you through this
% TP. Some functions might need a Computer Vision Toolbox to run. If they
% do not work, just comment them and perform the part which doesn't require
% any special toolbox.


% all the parts that should be filled in are marked by 'TO DO'

% use run and advance command to run each section!
%%
% Kinect sensor provides with 3 images, infra-red, color and depth image.
% Here we will work with depth and rgb images. Let's first load them:

% read depth image
% read color image
I_1 = imread('004378_color.png');
D_1=imread('004378_depth.png');

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
% ex  function filtered_depth = filterValues(D_1, thresh);
% thresh = 2500;
% 
%  TO DO
% show your filtered data
imshow(filtered_depth, []);
%% show point cloud (Computer Vision System toolbox is needed)
% Knowing the intrinsic parameters of a camera, we can always find real
% scene coordinates (X,Y,Z) using a depth image. 
% Here the calibration is performed beforehand, and the values of F_x, cx,
% cy are
% Please read more on camera calibration if you are interested
% Tsai, R. (1987). A versatile camera calibration technique for high-accuracy 
% 3D machine vision metrology using off-the-shelf TV cameras and lenses. 
% IEEE Journal on Robotics and Automation, 3(4), 323-344.
pc=zeros([size(D_1) 3]);
W=size(D_1,2);
H=size(D_1,1);
cx_d = 256.265446519628880;
cy_d = 212.411466505718410;
fy_d = 357.304303134804570;
fx_d = 357.530142098200430;

for indWidth = 1:W
    for indHeight= 1:H

            pc(indHeight,indWidth,3) = D_1(indHeight,indWidth); %direct depth value 
			pc(indHeight,indWidth,1) = pc(indHeight,indWidth,3) * (indHeight - cx_d) / fx_d;
			pc(indHeight,indWidth,2) = pc(indHeight,indWidth,3) * (cy_d - indWidth) / fy_d;
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
% Copy the code above but change D_1 to your filtered image. 
% TO DO



%% adding a mask of  a user
% Kinect SDK also provides with a human detection features
% Lets load a binary mask of a user provided by kinect and stored
mask = imread('004378_mask.png');
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
Joints =[];
M = dlmread('JointsInfo_Cyrille_v2_2.txt');
[t,d] = size(M);
Orientation_X = M(:,6:8:end); %orientations of each joint
Orinetation_Y = M(:,7:8:end);
Orientation_Z = M(:,8:8:end);
Orientation_W = M(:,9:8:end);
State = M(2:8:d); % if joint was estimated correctly or not
 
X = M(:,3:8:end);
Y = M(:,4:8:end);
Z = M(:,5:8:end);

%%
%% Pre-processing of the data
% As  a  common  preprocessing  steps,  we  compute  the  relative  
% difference  of  each  joint’  triplets [xi(t), yi(t), zi(t)]?
% with the position of the root joint [xroot(t), yroot(t), zroot(t)]? for any t . 
% root is the hip joint --> we take the spin base joint
X=bsxfun(@minus,X,X(:,1));
Y=bsxfun(@minus,Y,Y(:,1));
Z=bsxfun(@minus,Z,Z(:,1));


%%
% filtering
% Filtering is a common pre-processing operation for the kinect joints
% data.
% please, filter the X, Y and Z separately using a median filter averaging
% 7 frames. Store the data into X, Y and Z;
% Hint: use medfilt1
% TO DO

%%
% reshape joints to be able to plot the data
% bonus task - this is not optimal to calculate, since matlab works with an
% array of changing size - rewrite this procedure more optimal if you are
% interested
% measure the time it takes for my implementation and your alternative one.

X1=[]; Y1=[]; Z1=[];
for i=1:t
 X1=[X1, X(i,:)];
 Y1=[Y1, Y(i,:)];
 Z1=[Z1, Z(i,:)];
end

XYZ=[X1', Y1', Z1'];

PlotData(XYZ);


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
% Here we will try a simplified case. Remember the TD!
% use the fx_d, fy_d, cx_d and cy_d given above.



% Now project the joints from the first frame to the color image  (3D-2D)
% HINT: check the 2d-3d projection above!
% HINT2: use the unnormilized (not processed joints)

% first_frame_3d= XYZ_non_normalized(1:25,:);
first_frame_2d=zeros(25,2);
cx_d = 256.265446519628880;
cy_d = 212.411466505718410;
fy_d = 357.304303134804570;
fx_d = 357.530142098200430;
% TO DO 

% How to check: The projected joints are already on the color image -
% compare with the coordinates obtained 
% PS you won't get great results, because we are not considering here the
% FOV for the simplicity. But you should get the projections within the
% image
figure
imshow(I_1); %imshow color
hold on
scatter(first_frame_2d(:,1), first_frame_2d(:,2), 'r'); % plot your 2d coordinates


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


