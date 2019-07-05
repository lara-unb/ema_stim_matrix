%
% MATLAB code - EMA Matrix Experiments
% 2019-07-05
% Lucas de Macedo Pinheiro
% 
%   Find the best fitting line for force data, plot and save.
%
%


ForceNorm = Force;
ForceNorm.Data = ForceNorm.Data/max(ForceNorm.Data);

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
