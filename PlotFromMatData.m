%
%
% MATLAB code - EMA Matrix Experiments
% 2019-05-19
% Lucas de Macedo Pinheiro
% 
%   Plot the stimulator, current and force data 
% from mat files.
%
%
Files = dir('MATLAB*Right*.mat');

for w = 1:length(Files)
    
    load(Files(w).name);

    figure;
    plot(Force);
    hold on
    plot(StimCurrent./40);
    title(Files(w).name(8:end-4))
    ylim([-0.5 2.5])
    hold off
end

Files = dir('MATLAB*Left*.mat');

for w = 1:length(Files)
    
    load(Files(w).name);
    
    figure;
    plot(Force);
    hold on
    plot(StimCurrent./40);
    title(Files(w).name(8:end-4))
    ylim([-0.5 2.5])
    
    %shade = area([StimCommandData.Time(100),StimCommandData.Time(400)],[10,10],2);
    %shade.FaceColor = 'k'; shade.FaceAlpha = 0.3; shade.EdgeAlpha = 0.3;
    
    hold off
end