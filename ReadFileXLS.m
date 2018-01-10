function [ data_kinect, data_opto ] = ReadFileXLS( name )
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here

%name='pp1_trial1.xls';
data = xlsread(name,'Sheet1');

data_kinect=data(:,1:57);
data_opto=data(:,58:114);


%get rid of all the NAN values
data_kinect(isnan(data_kinect)) = 0.0;
data_opto(isnan(data_opto)) = 0.0;
end

