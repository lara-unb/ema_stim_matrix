%
% MATLAB code - EMA Matrix Experiments
% 2019-07-04
% Lucas de Macedo Pinheiro
% 
%   Integrate the EMG x Time graph and find fitting line.
%
%

clear; close all;

% Open window for file selection
disp('Select the MAT files...');
Files = uigetfile('*.mat','Select The MAT Files','MultiSelect','on');
if isa(Files,'char') % Only one file selected
   Files = {Files}; 
end

EMGExcludeIdx = [];
% EMGExcludeIdx = [1,2];

%%
for w = 1:length(Files)
% Import MAT file
    fprintf('\n\n%d/%d Loading "%s" file...\n',w,length(Files),Files{w});
      load(Files{w});

%% Normalize the force data
    disp('Normalizing...');
    EMGNorm = EMGdata;
%     EMGNorm.Data = EMGNorm.Data/max(EMGNorm.Data);

    [PeakValues,PeakIdx] = findpeaks(EMGNorm.Data,...
        'MinPeakDistance',50,'MinPeakHeight',0.02);
    PeakValues(EMGExcludeIdx) = [];
    EMGNorm.Data = EMGNorm.Data/max(PeakValues);
    
%     EMGNorm.Data = EMGNorm.Data/max(EMGNorm.Data(229:end));
    
%% Calculate the force integral during stimulation sequences
    disp('Calculating areas...');
    StimEdges = find(diff(StimCommandZeroed.Time)>4); % Edge threshold
    StimEdges = sort([StimEdges; StimEdges+1]); % Both rising and falling edges
    StimEdges = StimCommandZeroed.Time([1; StimEdges; end]); % Get time

    CompareM = pdist2(EMGNorm.Time,StimEdges); % Compares each element
    [~,AllIndicesVec] = min(CompareM); % Get index of closest time
    IndexM = vec2mat(AllIndicesVec,2); % Column1 is start, column2 is end

    EMGStimCell = cell(length(IndexM),1); % Cell with the 36 timeXforce data
    EMGAreasVec = zeros(length(IndexM),1);
    for i=1:length(IndexM)
        IdxVec = IndexM(i,1):IndexM(i,2);
        EMGStimCell{i,1} = [EMGNorm.Time(IdxVec) EMGNorm.Data(IdxVec)];
        EMGAreasVec(i,1) = trapz(EMGStimCell{i}(:,1),EMGStimCell{i,1}(:,2));
    end
%     EMGMidStimTimes = EMGNorm.Time(round(mean(IndexM,2)));
    EMGMidStimTimes = (7.5:10:357.5)';

%% Perform data fitting w/ poly1 (a*x + b)
%
% Default:
%	Normalize:'off', Exclude:[], Weights:[], Method:'NonlinearLeastSquares',
%   Robust:'Off', StartPoint:[1?0 double], Lower:[1?0 double],
%   Upper:[1?0 double], Algorithm:'Trust-Region', DiffMinChange:1.0000e-08,
%   DiffMaxChange:0.1000, Display:'Notify', MaxFunEvals:600, MaxIter:400,
%   TolFun:1.0000e-06, TolX: 1.0000e-06

    disp('Fitting line...');
    [EMGCurveFitData,EMGCurveFitStats,~] = fit(EMGMidStimTimes,EMGAreasVec,'poly1',...
        'Exclude', EMGExcludeIdx);
    EMGAreasVecValid = EMGAreasVec;
    EMGAreasVecValid(EMGExcludeIdx) = [];
    EMGCurveFitCoefs = coeffvalues(EMGCurveFitData); % Get a and b
    EMGCurveFitCI = confint(EMGCurveFitData); % Get a and b 95% intervals
    EMGCurveFitEq = sprintf('%.3et %+ .3f',EMGCurveFitCoefs);
    fprintf('COEF: %.3f(%.3f,%.3f)\n',1e3*EMGCurveFitCoefs(1),1e3*EMGCurveFitCI(1,1),1e3*EMGCurveFitCI(2,1));
    eTI.mean = mean(EMGAreasVecValid);
    eTI.std = std(EMGAreasVecValid);
    fprintf('eTI (Mean\x00B1SD): %.3f\x00B1%.3f\n',eTI.mean,eTI.std);

%% Plot data
    figure
    hold on
    yyaxis left % Left Y axis 
    plot(EMGNorm)
    ylabel('eEMG Amplitude (mV)'), xlim([-10 380])
    title(Files{w},'Interpreter', 'none')
    LimitsL = [-.19 2]; ylim(LimitsL); box on;
    yyaxis right  % Right Y axis 
    
    if ~isempty(EMGExcludeIdx)
        ExcludeVec = false(1,length(EMGAreasVec));
        ExcludeVec(EMGExcludeIdx) = true;
        plot(EMGCurveFitData,EMGMidStimTimes,EMGAreasVec,'o',ExcludeVec,'x')
        legend('eEMG','eTI','Excluded','eTI Fit','Location','Best')
    else
        plot(EMGCurveFitData,EMGMidStimTimes,EMGAreasVec,'o')
        legend('eEMG','eTI','eTI Fit','Location','Best')
    end
    
    LimitsR = [0 4.2];
    ylim([LimitsL(1)*LimitsR(2)/LimitsL(2) LimitsR(2)])
    ylabel('eEMG-Time Integral (mV.s)'), xlabel('Time (s)')
    hold off

%% Save data 
    disp('Saving figure and file...');
    savefig([Files{w} '_EMG_Fitted.fig']);
    
    FileStruct.(Files{w}).('EMGNorm') = EMGNorm;
    FileStruct.(Files{w}).('EMGExcludeIdx') = EMGExcludeIdx;
    FileStruct.(Files{w}).('EMGMidStimTimes') = EMGMidStimTimes;
    FileStruct.(Files{w}).('EMGAreasVec') = EMGAreasVec;
    FileStruct.(Files{w}).('EMGCurveFitData') = EMGCurveFitData;
    FileStruct.(Files{w}).('EMGCurveFitStats') = EMGCurveFitStats;
    FileStruct.(Files{w}).('EMGAreasVecValid') = EMGAreasVecValid;
    FileStruct.(Files{w}).('EMGCurveFitCoefs') = EMGCurveFitCoefs;
    FileStruct.(Files{w}).('EMGCurveFitCI') = EMGCurveFitCI;
    FileStruct.(Files{w}).('EMGCurveFitEq') = EMGCurveFitEq;
    FileStruct.(Files{w}).('eTI') = eTI;
    
end


