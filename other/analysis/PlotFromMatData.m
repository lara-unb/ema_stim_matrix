%
% MATLAB code - EMA Matrix Experiments
% 2019-05-19
% Lucas de Macedo Pinheiro
% 
%   Plot the stimulator and force data from mat files.
%
%

clear; close all;

% Open window for file selection
disp('Select the MAT files...');
Files = uigetfile('*.mat','Select The MAT Files','MultiSelect','on');
if isa(Files,'char') % Only one file selected
   Files = {Files}; 
end

Original = ;

for w = 1:length(Files)
    load(Files{w});
    
    figure
    plot(EMGdata)
    hold on
%     plot(StimCurrent.Time, StimCurrent.Data./40)
    xlim([-10 380])
    ylabel('eEMG (mV)'), xlabel('Time (s)')
    title 'E2_SUBX1_2M_Left'
    hold off
    
    savefig([Files{w}(15:end-4) '.fig']);
end
