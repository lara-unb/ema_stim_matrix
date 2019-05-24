%
%
% MATLAB code - EMA Matrix Experiments
% 2019-05-16
% Lucas de Macedo Pinheiro
% 
%   Parse the bag files and save stimulator,
% current and force data to different mat files.
%
%

% Right leg bag files
fprintf('\n\nRIGHT LEG DATA\n');
Files = dir('Matrix*Right*.bag');

for w = 1:length(Files)

    % Import bag file
    fprintf('\n\nImporting "%s" bag file...\n', Files(w).name);
    RawBag = rosbag(Files(w).name);

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

    % Save force and stim data to file
    disp('Saving MAT file...');
    save(['MATLAB_' Files(w).name(1:end-4)],'StimCommandData','ForceTrim',...
         'StimCurrentTrim');
end

% Left leg bag files
fprintf('\n\nLEFT LEG DATA\n');
Files = dir('Matrix*Left*.bag');

for w = 1:length(Files)

    % Import bag file
    fprintf('\n\nImporting "%s" bag file...\n', Files(w).name);
    RawBag = rosbag(Files(w).name);

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

    % Save force and stim data to file
    disp('Saving MAT file...');
    save(['MATLAB_' Files(w).name(1:end-4)],'StimCommandData','ForceTrim',...
         'StimCurrentTrim');
end