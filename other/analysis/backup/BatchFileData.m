%
% MATLAB code - EMA Matrix Experiments
% 2020-01-25
% Lucas de Macedo Pinheiro
% 
%   Import mat files and store in a struct.
%
%

clear; close all;

% Open window for file selection
disp('Select the MAT files...');
Files = uigetfile('*.mat','Select The MAT Files','MultiSelect','on');
if isa(Files,'char') % Only one file selected
   Files = {Files}; 
end

%% Import .mat files and append Linear Model w/ SD
FileStruct = struct;

for w = 1:length(Files)
    fprintf('\n\n%d/%d Loading "%s" file...\n',w,length(Files),Files{w});
    FileStruct.(Files{w}(15:end-7)) = load(Files{w});
    g = FileStruct.(Files{w}(15:end-7));
    FileStruct.(Files{w}(15:end-7)).LinearModelFit = fitlm(g.MidStimTimes,g.AreasVec,'Exclude',g.ExcludeIdx);
end

disp('Saving file...');
save([Files{1}(18:22) '_AllData'],'FileStruct','-append'); % '-v7'

%% Dump to .txt 
Export = fopen('out.txt','w','n','utf-8');
Fields = fieldnames(FileStruct);
    
for k = 1:numel(Fields)
    g = FileStruct.(Fields{k});
    fprintf(Export,'\n%s\n',Fields{k});
    fprintf(Export,'COEF(CI)e-3:\t %.3f(%.3f,%.3f)\n',...
        1e3*g.CurveFitCoefs(1),1e3*g.CurveFitCI(1,1),1e3*g.CurveFitCI(2,1));
    fprintf(Export,'FTI(\x00B1SD):\t %.3f\x00B1%.3f\n',g.FTI.mean,g.FTI.std);    
    fprintf(Export,'Eq:\t\t y = %s\n',g.CurveFitEq);    
    fprintf(Export,'LMSlope(\x00B1SD)e-3: %.3f\x00B1%.3f\n',...
        1e3*g.LinearModelFit.Coefficients.Estimate(2),1e3*g.LinearModelFit.Coefficients.SE(2));
    fprintf(Export,'LMBias(\x00B1SD):\t %.3f\x00B1%.3f\n\n',...
        g.LinearModelFit.Coefficients.Estimate(1),g.LinearModelFit.Coefficients.SE(1));
end

%% Plot ForceAreaFitting
Fields = fieldnames(FileStruct);

for k = 1:numel(Fields)
    
    g = FileStruct.(Fields{k});

% Plot data
    figure
    hold on
    yyaxis left % Left Y axis 
    plot(g.ForceNorm)
    ylabel('Force (kgf)'), xlim([-10 380])
    title(Fields{k},'Interpreter', 'none')
    LimitsL = [-.19 2]; ylim(LimitsL); box on;
    yyaxis right  % Right Y axis 
    
    if ~isempty(g.ExcludeIdx)
        ExcludeVec = false(1,length(g.AreasVec));
        ExcludeVec(g.ExcludeIdx) = true;
        plot(g.CurveFitData,g.MidStimTimes,g.AreasVec,'o',ExcludeVec,'x')
        legend('Force','FTI','Excluded','FTI Fit','Location','Best')
    else
        plot(g.CurveFitData,g.MidStimTimes,g.AreasVec,'o')
        legend('Force','FTI','FTI Fit','Location','Best')
    end
    
    LimitsR = [0 6.2];
    ylim([LimitsL(1)*LimitsR(2)/LimitsL(2) LimitsR(2)])
    ylabel('Force-Time Integral (kgf.s)'), xlabel('Time (s)')
    hold off

end

