%
% MATLAB code - EMA Matrix Experiments
% 2019-05-16
% Lucas de Macedo Pinheiro
%

close all
clear all

% Import bag file
disp('Importing bag file...');
RawBag = rosbag('Matrix_Pilot_1M_Right_S1.bag');

% Separate stim command data
disp('Extracting stim command data from bag file...');
StimCommandTopicFromBag = select(RawBag,'Topic','/ema/stimulator/single_pulse');
StimCommandData = cell2table(readMessages(StimCommandTopicFromBag));
StimCommandData = table(StimCommandData(:,1),'VariableNames',{'Data'});
StimCommandData.Time = StimCommandTopicFromBag.MessageList.Time;

% Separate stim current data
disp('Extracting stim current data from bag file...');
StimCurrentTopicFromBag = select(RawBag,'Topic','/ema/matrix/stimsignal');
StimCurrentTS = timeseries(StimCurrentTopicFromBag);

% Separate raw wrench force data
disp('Extracting raw force data from bag file...');
RawForceTopicFromBag = select(RawBag,'Topic','/ema/forcesensor/loadcell');
RawForceTS = timeseries(RawForceTopicFromBag);
% Get the actual timestamp from sensor data (secs + nanosecs)
ForceTimestampVec = RawForceTS.Data(:,2) + RawForceTS.Data(:,3)*1e-09;
ForceValues = RawForceTS.Data(:,5);
ForceTS = timeseries(ForceValues,ForceTimestampVec);

% Trim data to get the important part
disp('Trimming data...');
ti = StimCommandData.Time(1)-5; % 5s before stim starts
tf = StimCommandData.Time(end)+5; % 5s after stim ends
% Maintaining Time Series
ForceTrim = getsamples(ForceTS,...
    find(ForceTS.Time>ti,1):find(ForceTS.Time<tf,1,'last'));
StimCurrentTrim = getsamples(StimCurrentTS,...
    find(StimCurrentTS.Time>ti,1):find(StimCurrentTS.Time<tf,1,'last'));

figure;
plot(ForceTrim);
hold on
plot(StimCurrentTrim./40);
hold off