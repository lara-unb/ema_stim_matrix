%
% MATLAB code - EMA Matrix Experiments
% 2019-05-27
% Lucas de Macedo Pinheiro
% 
%   Parse the bag files, process data and save stimulator, current and 
% force data to mat files.
%

clear; close all;

% Open window for file selection
disp('Select the bagfiles...');
Files = uigetfile('*.bag','Select The Bagfiles','MultiSelect','on');
if isa(Files,'char') % Only one file selected
   Files = {Files}; 
end

for w = 1:length(Files)
% Import bag file
    fprintf('\n\nImporting "%s" bag file...\n', Files{w});
    RawBag = rosbag(Files{w});

% Separate stim command data
    disp('Extracting stim command data from bag file...');
    StimCommandTopicFromBag = select(RawBag,'Topic','/ema/stimulator/single_pulse');
    StimCommandRaw = cell2table(readMessages(StimCommandTopicFromBag));
    StimCommandRaw = table(StimCommandRaw(:,1),'VariableNames',{'Data'});
    StimCommandRaw.Time = StimCommandTopicFromBag.MessageList.Time;

% Separate stim current data
    disp('Extracting stim current data from bag file...');
    StimCurrentTopicFromBag = select(RawBag,'Topic','/ema/matrix/stimsignal');
    StimCurrentRaw = timeseries(StimCurrentTopicFromBag);

% Separate raw wrench force data
    disp('Extracting raw force data from bag file...');
    RawForceTopicFromBag = select(RawBag,'Topic','/ema/forcesensor/loadcell');
    RawForceTS = timeseries(RawForceTopicFromBag);
    % Get the actual timestamp from sensor data (secs + nanosecs)
    ForceTimestampVec = RawForceTS.Data(:,2) + RawForceTS.Data(:,3)*1e-09;
    ForceValues = RawForceTS.Data(:,5);
    ForceRaw = timeseries(ForceValues,ForceTimestampVec);

% Trim data to get the important part
    disp('Trimming data...');
    ti = StimCommandRaw.Time(1)-5; % 5s before stim starts
    tf = StimCommandRaw.Time(end)+5; % 5s after stim ends
    % Maintaining Time Series
    ForceTrim = getsamples(ForceRaw,...
        find(ForceRaw.Time>ti,1):find(ForceRaw.Time<tf,1,'last'));
    StimCurrentTrim = getsamples(StimCurrentRaw,...
        find(StimCurrentRaw.Time>ti,1):find(StimCurrentRaw.Time<tf,1,'last'));

% Adjust the time to begin with zero
    disp('Adjusting start time...');
    Toffset = min([ForceTrim.Time(1),StimCurrentTrim.Time(1)]);
    ForceTrim.Time = ForceTrim.Time - Toffset;
    StimCurrentTrim.Time = StimCurrentTrim.Time - Toffset;
    StimCommandZeroed = StimCommandRaw;
    StimCommandZeroed.Time = StimCommandRaw.Time - Toffset;

% Save data to file
    Force = ForceTrim;
    StimCurrent = StimCurrentTrim;
    StimCommand = StimCommandZeroed;
    
    disp('Saving MAT file...');
    save(['MATLAB_' Files{w}(1:end-4)],...
        'ForceRaw','StimCurrentRaw','StimCommandRaw',...
        'ForceTrim','StimCurrentTrim','StimCommandZeroed',...
        'Force','StimCurrent','StimCommand','-v7');
end
