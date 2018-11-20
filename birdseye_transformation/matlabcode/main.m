clear all
close all
clc

% to load different picture, change code in line 29

%% load camera parameters and distortion model

K = load('calibrations/K.txt');
projMat = load('calibrations/projectionmatrix.txt');
R_T = inv(K)*projMat;
R = R_T(1:3,1:3);
T = R_T(1:3,4);
focalLength = [K(1,1) K(2,2)];

% K transpose (MATLAB convention)
K = K';

rDist = load('calibrations/radialdistortion.txt');
tDist = load('calibrations/tangentialdistortion.txt');

cameraParams = cameraParameters('IntrinsicMatrix',K,...
                                'RadialDistortion',rDist,...
                                'TangentialDistortion',tDist,...
                                'ImageSize',[420 680]);

%% load picture and find original size and centerpoint

orgimg = imread('selectedsamples/300.png');
orgimgsize =  size(orgimg);
orgimgmid = round(orgimgsize(1:2)./2);

%% undistort image

[udistimg,newOrigin] = undistortImage(orgimg,cameraParams); %, 'OutputView', 'full');
udistimgsize =  size(udistimg);
udistimgmid = round(udistimgsize(1:2)./2);

%% create sensor data

%intrinsics = cameraIntrinsics(focalLength,[317.9700  247.4800],udistimgsize(1:2));
height = 0.1; % height of camera above floor
pitch = 12; % camera angle relative to plane

sensor = monoCamera(cameraParams,height,'Pitch',pitch);
% pixl size of output image (length computed automatically (NaN)
outImageSize = [NaN,640];
% field of view measurements
distAhead = 1;
spaceToOneSide = 0.5;
bottomOffset = 0;
outView = [bottomOffset,distAhead,-spaceToOneSide,spaceToOneSide];
birdsEye = birdsEyeView(sensor,outView,outImageSize);
birdimg = transformImage(birdsEye,udistimg);

% plot original, undistorted and birdseye image
subplot(1,3,1)
imshow(orgimg)
subplot(1,3,2)
imshow(udistimg)
subplot(1,3,3)
imshow(birdimg)
