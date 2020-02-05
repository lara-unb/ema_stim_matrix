%
% MATLAB code - EMA Matrix Experiments
% 2020-01-25
% Lucas de Macedo Pinheiro
% 
%   Sync EMG signal with stimulation.
%
%

k=7;

%%
g = FileStruct.(Fields{k});
figure
plot(g.EMGdata)
hold on
plot(g.StimCurrent.Time, g.StimCurrent.Data./400)
%%
ti = g.StimCommandZeroed.Time(1)-5; % 5s before stim starts
tf = g.StimCommandZeroed.Time(end)+5; % 5s after stim ends
% Maintaining Time Series
FileStruct.(Fields{k}).EMGdata = getsamples(g.EMGdata,...
    find(g.EMGdata.Time>ti,1):find(g.EMGdata.Time<tf,1,'last'));
%%
for k=2:8
    figure
    plot(FileStruct.(Fields{k}).EMGdata)
    hold on
    plot(FileStruct.(Fields{k}).StimCurrent.Time, FileStruct.(Fields{k}).StimCurrent.Data./400)
    hold off
end

