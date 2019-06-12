%
%
% MATLAB code - EMA Matrix Experiments
% 2019-06-10
% Lucas de Macedo Pinheiro
% 
%   Find the best power function to fit the data
% plot and save it.
%

clear; close all;
file = 'MATLAB_Matrix_SUB01_1M_Right_S1';
load(file);

Force.Data = Force.Data/max(Force.Data);
ForceNorm = Force;
[PeakValues,PeakIdx] = findpeaks(Force.Data,'MinPeakDistance',100);
PeakTimes = Force.Time(PeakIdx);

[CurveFitData,~,~] = fit(PeakTimes,PeakValues,'poly1'); % poly1 = a*x + b
% CurveFitCoefs = coeffvalues(CurveFitData);
% CurveFitEq = [num2str(CurveFitCoefs(1),3) '?x^' num2str(CurveFitCoefs(2),3)];

figure
plot(Force)
hold on
plot(PeakTimes,PeakValues,'o')
plot(CurveFitData)
xlim([0 380])
ylim([-0.2 1.2])
title(file)
xlabel('Time (s)')
ylabel('Force')
legend('Normalized Force','Peaks', 'Linear Fitting')

% save(file);