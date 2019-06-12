%
%
% MATLAB code - EMA Matrix Experiments
% 2019-05-19
% Lucas de Macedo Pinheiro
% 
%   Plot the stimulator and force data from mat files.
%

Files = dir('MATLAB*Right*.mat');
for w = 1:length(Files)
    load(Files(w).name);
    figure
    plot(Force)
    hold on
    plot(StimCurrent.Time, StimCurrent.Data./40)
    title(Files(w).name(8:end-4))
    xlabel('Time (s)')
    ylabel(' ')
    xlim([-10 380])
    legend('Force (kg)','Stimulation (y/n)')
    hold off
    savefig([Files(w).name(8:end-4) '.fig']);
end

Files = dir('MATLAB*Left*.mat');
for w = 1:length(Files)
    load(Files(w).name);
    figure
    plot(Force)
    hold on
    plot(StimCurrent.Time, StimCurrent.Data./40)
    title(Files(w).name(8:end-4))
    xlabel('Time (s)')
    ylabel(' ')
    xlim([-10 380])
    legend('Force (kg)','Stimulation (y/n)')
    hold off
    savefig([Files(w).name(8:end-4) '.fig']);
end
