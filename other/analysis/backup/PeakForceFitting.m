%
% MATLAB code - EMA Matrix Experiments
% 2019-06-10
% Lucas de Macedo Pinheiro
% 
%   Find the best fitting line for force data, plot and save.
%
%

clear; close all;

% Open window for file selection
disp('Select the MAT files...');
Files = uigetfile('*.mat','Select The MAT Files','MultiSelect','on');
if isa(Files,'char') % Only one file selected
   Files = {Files}; 
end

for w = 1:length(Files)
% Import MAT file
    fprintf('\n\nLoading "%s" file...\n', Files{w});
    load(Files{w});
    
% Normalize the force data
    disp('Normalizing force...');
    ForceNorm = Force;
    ForceNorm.Data = ForceNorm.Data/max(ForceNorm.Data);
    
% Find the peaks from stimulating sequences
    disp('Finding peaks...');
    [PeakValues,PeakIdx] = findpeaks(ForceNorm.Data,...
        'MinPeakDistance',50,'MinPeakHeight',0.2);
    PeakTimes = ForceNorm.Time(PeakIdx);
    
% Perform data fitting w/ poly1 (a*x + b)
    disp('Fitting curve...');
    [CurveFitData,~,~] = fit(PeakTimes,PeakValues,'poly1');
    CurveFitCoefs = coeffvalues(CurveFitData); % Get a and b

    figure
    plot(ForceNorm)
    hold on
    plot(StimCurrent.Time, StimCurrent.Data./40,'color',[0,0,0,.5])
    plot(PeakTimes,PeakValues,'o')
    plot(CurveFitData)
    xlim([0 380]), ylim([-0.2 1.3])
    ylabel(' '), xlabel('Time (s)'), title(Files{w}(15:end-4))
    legend('Normalized Force','Stimulation','Peaks','Linear Fitting')
    hold off

    disp('Saving figure and file...');
    savefig([Files{w}(15:end-4) '_Fitted.fig']);
    save(Files{w});
end
