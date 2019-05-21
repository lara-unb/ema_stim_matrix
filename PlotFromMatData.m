%
% MATLAB code - EMA Matrix Experiments
% 2019-05-16
% Lucas de Macedo Pinheiro
%

    figure;
    plot(ForceTrim);
    hold on
    plot(StimCurrentTrim./40);
    title(Files(w).name)
    ylim([-0.5 2.5])
    hold off