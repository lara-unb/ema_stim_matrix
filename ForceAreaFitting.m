%
% MATLAB code - EMA Matrix Experiments
% 2019-07-04
% Lucas de Macedo Pinheiro
% 
%   Integrate the Force x Time graph and find fitting line.
%
%

clear; close all;

% Open window for file selection
disp('Select the MAT files...');
Files = uigetfile('*.mat','Select The MAT Files','MultiSelect','on');
if isa(Files,'char') % Only one file selected
   Files = {Files}; 
end

ExcludeIdx = [];
% ExcludeIdx = [1,2];

%%
for w = 1:length(Files)
% Import MAT file
    fprintf('\n\n%d/%d Loading "%s" file...\n',w,length(Files),Files{w});
      load(Files{w});

%% Normalize the force data
    disp('Normalizing force...');
    ForceNorm = Force;
%     ForceNorm.Data = ForceNorm.Data/max(ForceNorm.Data);

    [PeakValues,PeakIdx] = findpeaks(ForceNorm.Data,...
        'MinPeakDistance',50,'MinPeakHeight',0.2);
    PeakValues(ExcludeIdx) = [];
    ForceNorm.Data = ForceNorm.Data/max(PeakValues);
    
%     ForceNorm.Data = ForceNorm.Data/max(ForceNorm.Data(229:end));
    
%% Calculate the force integral during stimulation sequences
    disp('Calculating areas...');
    StimEdges = find(diff(StimCommandZeroed.Time)>4); % Edge threshold
    StimEdges = sort([StimEdges; StimEdges+1]); % Both rising and falling edges
    StimEdges = StimCommandZeroed.Time([1; StimEdges; end]); % Get time

    CompareM = pdist2(ForceNorm.Time,StimEdges); % Compares each element
    [~,AllIndicesVec] = min(CompareM); % Get index of closest time
    IndexM = vec2mat(AllIndicesVec,2); % Column1 is start, column2 is end

    ForceStimCell = cell(length(IndexM),1); % Cell with the 36 timeXforce data
    AreasVec = zeros(length(IndexM),1);
    for i=1:length(IndexM)
        IdxVec = IndexM(i,1):IndexM(i,2);
        ForceStimCell{i,1} = [ForceNorm.Time(IdxVec) ForceNorm.Data(IdxVec)];
        AreasVec(i,1) = trapz(ForceStimCell{i}(:,1),ForceStimCell{i,1}(:,2));
    end
%     MidStimTimes = ForceNorm.Time(round(mean(IndexM,2)));
    MidStimTimes = (7.5:10:357.5)';

%% Perform data fitting w/ poly1 (a*x + b)
%
% Default:
%	Normalize:'off', Exclude:[], Weights:[], Method:'NonlinearLeastSquares',
%   Robust:'Off', StartPoint:[1?0 double], Lower:[1?0 double],
%   Upper:[1?0 double], Algorithm:'Trust-Region', DiffMinChange:1.0000e-08,
%   DiffMaxChange:0.1000, Display:'Notify', MaxFunEvals:600, MaxIter:400,
%   TolFun:1.0000e-06, TolX: 1.0000e-06

    disp('Fitting line...');
    [CurveFitData,CurveFitStats,~] = fit(MidStimTimes,AreasVec,'poly1',...
        'Exclude', ExcludeIdx);
    AreasVecValid = AreasVec;
    AreasVecValid(ExcludeIdx) = [];
    CurveFitCoefs = coeffvalues(CurveFitData); % Get a and b
    CurveFitCI = confint(CurveFitData); % Get a and b 95% intervals
    CurveFitEq = sprintf('%.3et %+ .3f',CurveFitCoefs);
    fprintf('COEF: %.3f(%.3f,%.3f)\n',1e3*CurveFitCoefs(1),1e3*CurveFitCI(1,1),1e3*CurveFitCI(2,1));
    FTI.mean = mean(AreasVecValid);
    FTI.std = std(AreasVecValid);
    fprintf('FTI (Mean\x00B1SD): %.3f\x00B1%.3f\n',FTI.mean,FTI.std);

%% Plot data
    figure
    hold on
    yyaxis left % Left Y axis 
    plot(ForceNorm)
    ylabel('Force (kgf)'), xlim([-10 380])
    title(Files{w}(1:end-4),'Interpreter', 'none')
    LimitsL = [-.19 2]; ylim(LimitsL); box on;
    yyaxis right  % Right Y axis 
    
    if ~isempty(ExcludeIdx)
        ExcludeVec = false(1,length(AreasVec));
        ExcludeVec(ExcludeIdx) = true;
        plot(CurveFitData,MidStimTimes,AreasVec,'o',ExcludeVec,'x')
        legend('Force','FTI','Excluded','FTI Fit','Location','Best')
    else
        plot(CurveFitData,MidStimTimes,AreasVec,'o')
        legend('Force','FTI','FTI Fit','Location','Best')
    end
    
    LimitsR = [0 6.2];
    ylim([LimitsL(1)*LimitsR(2)/LimitsL(2) LimitsR(2)])
    ylabel('Force-Time Integral (kgf.s)'), xlabel('Time (s)')
    hold off

%% Save data 
    disp('Saving figure and file...');
    savefig([Files{w}(1:end-4) '_Fitted.fig']);
    save(Files{w},'ForceNorm','ExcludeIdx','MidStimTimes','AreasVec',...
        'CurveFitData','CurveFitStats','AreasVecValid','CurveFitCoefs',...
        'CurveFitCI','CurveFitEq','FTI','-append'); % '-v7'
end


