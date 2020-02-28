%
% MATLAB code - EMA Matrix Experiments
% 2019-07-18
% Lucas de Macedo Pinheiro
% 
%   Gather fit data from mat files.
%
%

clear; close all;

% Open window for file selection
disp('Select the MAT files...');
Files = uigetfile('*.mat','Select The MAT Files','MultiSelect','on');
if isa(Files,'char') % Only one file selected
   Files = {Files}; 
end
%%
for w = 1:length(Files)
% Import MAT file
    fprintf('\n\nLoading "%s" file...\n', Files{w});
    TempStruct.(['F' num2str(w)]) = load(Files{w});
end
%%
for w = 1:length(Files)
    fprintf('\n\n"%s" file...\n', Files{w});

    CurveFitCoefs = TempStruct.(['F' num2str(w)]).CurveFitCoefs;
    CurveFitCI = TempStruct.(['F' num2str(w)]).CurveFitCI;
    FTI = TempStruct.(['F' num2str(w)]).FTI;
    R2 = TempStruct.(['F' num2str(w)]).CurveFitStats.rsquare;
    RMSE = TempStruct.(['F' num2str(w)]).CurveFitStats.rmse;
    
    fprintf('COEF: %.3f(%.3f,%.3f)\n',1e3*CurveFitCoefs(1),1e3*CurveFitCI(1,1),1e3*CurveFitCI(2,1));
    fprintf('FTI (Mean?SD): %.3f?%.3f\n',FTI.mean,FTI.std);
    fprintf('R2: %.3f   RMSE: %.3f\n',R2,RMSE);
end








% clear; close all;
% 
% % Open window for file selection
% disp('Select the MAT files...');
% Files = uigetfile('*.mat','Select The MAT Files','MultiSelect','on');
% if isa(Files,'char') % Only one file selected
%    Files = {Files}; 
% end
% 
% for w = length(Files):-1:1
% % Import MAT file
%     fprintf('\n\nLoading "%s" file...\n', Files{w});
%     TempStruct.(['F' num2str(w)]) = load(Files{w});
% 
% % Construct a table with all the data
%     FileTable = struct2table(TempStruct.(['F' num2str(w)]),'AsArray',true);
%     FileName = {Files{w}(1:end-4)};
%     FileTable.ExcludeIdx = {FileTable.ExcludeIdx};
%     TempTable = [table(FileName), FileTable];
%     StatsTable(w,:) = TempTable; 
%     fprintf('COEF: %.3f(%.3f,%.3f)\n',1e3*StatsTable.CurveFitCoefs(w,1),1e3*StatsTable.CurveFitCI{w}(1,1),1e3*StatsTable.CurveFitCI{w}(2,1))
%     fprintf('FTI (Mean?SD): %.3f?%.3f\n',StatsTable.FTI(w).mean,StatsTable.FTI(w).std);
% end







% Parse the big table
% find(any(ismember(char(StatsTable.FileName{:,1}),'o0'),2))
% AllSES = StatsTable([2,3,5],:);

% Print for the tables
% fprintf('\nCOEF: %.3f(%.3f,%.3f)\n',1e3*AllSES.CurveFitCoefs(r,1),1e3*AllSES.CurveFitCI{r}(1,1),1e3*AllSES.CurveFitCI{r}(2,1))




