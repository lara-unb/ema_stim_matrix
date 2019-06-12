%
% MATLAB code - EMA Matrix Experiments
% 2019-05-27
% Lucas de Macedo Pinheiro
% 
%   Parse the bag files, process data and save stimulator,
% current and force data to mat files.
%

clear; close all;

% Uncomment these for each leg
fprintf('\n\nRIGHT LEG DATA\n');
Files = dir('Matrix*Right*.bag');
% fprintf('\n\nLEFT LEG DATA\n');
% Files = dir('Matrix*Left*.bag');

for w = 1:length(Files)

% Import bag file
    fprintf('\n\nImporting "%s" bag file...\n', Files(w).name);
    RawBag = rosbag(Files(w).name);

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
   
% % Get rid of trending with an average from non-stimulating phases
%     disp('Detrending data...');
%     % Find start and end indices of all zero sequences (non-stimulating phases)
%     zsig = ~~StimCurrentTrim.Data; % 0 for zeros, 1 for everyhting else
%     dsig = diff([1;zsig;1]); % 1 for rising edge, -1 for falling
%     SIndexVec = find(dsig < 0); % Vector w/ start indices
%     EIndexVec = find(dsig > 0)-1; % Vector w/ end indices
%     ValidVec = ((EIndexVec-SIndexVec) >= 1); % Get rid of small peaks
%     SIndexVec = SIndexVec(ValidVec); % Valid start indices
%     EIndexVec = EIndexVec(ValidVec); % Valid end indices
%     SortedTimes = StimCurrentTrim.Time(sort([SIndexVec;EIndexVec]));
%     CompareM = pdist2(ForceTrim.Time,SortedTimes); % Compares each element
%     [~,AllIndicesVec] = min(CompareM); % Get index of closest time
%     IndexM = vec2mat(AllIndicesVec,2); % Column1 is start, column2 is end
%     % Get the offset from start-end then apply it to the data
%     IndexM(end+1,1) = IndexM(end,2)+1; % Dealing with the last zero sequence
%     VecNoTrend = zeros(length(ForceTrim.Data),1); % Preallocating
%     for i=1:size(IndexM,1)-1
%         idxA10 = IndexM(i,1) + ceil(0.1*(IndexM(i,2)-IndexM(i,1)));
%         Foffset = mean(ForceTrim.Data(idxA10:IndexM(i,2)));
%         VecNoTrend(IndexM(i,1):(IndexM(i+1,1)-1)) = ...
%             ForceTrim.Data(IndexM(i,1):(IndexM(i+1,1)-1)) - Foffset;
%     end
%     ForceNoTrend = ForceTrim;
%     ForceNoTrend.Data = VecNoTrend;

% Save force and stim data to file

%     Force = ForceNoTrend;
    Force = ForceTrim;
    
    StimCurrent = StimCurrentTrim;
    StimCommand = StimCommandZeroed;
    
    disp('Saving MAT file...');
%     save(['MATLAB_' Files(w).name(1:end-4)],...
%           'ForceRaw','StimCurrentRaw','StimCommandRaw',...
%           'ForceTrim','StimCurrentTrim','StimCommandZeroed',...
%           'ForceNoTrend','Force','StimCurrent','StimCommand');
    save(['MATLAB_' Files(w).name(1:end-4)],...
          'ForceRaw','StimCurrentRaw','StimCommandRaw',...
          'ForceTrim','StimCurrentTrim','StimCommandZeroed',...
          'Force','StimCurrent','StimCommand');
end
