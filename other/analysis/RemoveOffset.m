%
% MATLAB code - EMA Matrix Experiments
% 2020-01-30
% Lucas de Macedo Pinheiro
% 
%   Identify and remove offset with an average preceding phase.
%   
% Finds start and end time of all no-stim phases in the 'StimCurrentTrim' 
% timeseries, calculates the offset/mean from 'Original' timeseries during 
% thoses phases, subtracts the mean from each of its corresponding no-stim
% phase and following stim phase and saves as NoTrend timeseries.
%
% Remember to change line 31 so that Original = <YourDataToDetrend>;
%

clear; close all;

%% Open window for file selection
disp('Select the MAT files...');
Files = uigetfile('*.mat','Select The MAT Files','MultiSelect','on');
if isa(Files,'char') % Only one file selected
   Files = {Files}; 
end

for w = 1:length(Files)
% Import MAT file
    fprintf('\n\nLoading "%s" file...\n', Files{w});
    load(Files{w});
    
    Original = EMGdata;
    
%% Find start and end indices of all zero sequences (non-stimulating phases)
    disp('Removing offset...');
    zsig = ~~StimCurrentTrim.Data; % 0 for zeros, 1 for everyhting else
    dsig = diff([1;zsig;1]); % 1 for rising edge, -1 for falling
    SIndexVec = find(dsig < 0); % Vector w/ start indices
    EIndexVec = find(dsig > 0)-1; % Vector w/ end indices
    ValidVec = ((EIndexVec-SIndexVec) >= 1); % Get rid of small peaks
    SIndexVec = SIndexVec(ValidVec); % Valid start indices
    EIndexVec = EIndexVec(ValidVec); % Valid end indices
    SortedTimes = StimCurrentTrim.Time(sort([SIndexVec;EIndexVec]));
    CompareM = pdist2(Original.Time,SortedTimes); % Compares each element
    [~,AllIndicesVec] = min(CompareM); % Get index of closest time
    IndexM = vec2mat(AllIndicesVec,2); % Column1 is start, column2 is end
    
% Get the offset from start-end then apply it to the data
    IndexM(end+1,1) = IndexM(end,2)+1; % Treating the last zero sequence
    VecNoTrend = zeros(length(Original.Data),1); % Preallocating
    for i=1:size(IndexM,1)-1
        idxA10 = IndexM(i,1) + ceil(0.1*(IndexM(i,2)-IndexM(i,1)));
        Foffset = mean(Original.Data(idxA10:IndexM(i,2)));
        VecNoTrend(IndexM(i,1):(IndexM(i+1,1)-1)) = ...
            Original.Data(IndexM(i,1):(IndexM(i+1,1)-1)) - Foffset;
    end
    NoTrend = Original;
    NoTrend.Data = VecNoTrend;

    figure
    plot(Original)
    hold on
    plot(NoTrend)
    xlim([0 380])
    ylabel(' '), xlabel('Time (s)'), title(Files{w}(15:end-4))
    legend('Before','After')
    hold off

%% Save/append data to file
    FinalData = NoTrend;
    
    disp('Saving MAT file...');
    save(Files{w},'FinalData','NoTrend','-append'); 
end
