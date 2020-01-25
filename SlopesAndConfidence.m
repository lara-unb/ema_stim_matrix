%
% MATLAB code - EMA Matrix Experiments
% 2020-01-25
% Lucas de Macedo Pinheiro
% 
%   Plot the slope and errorbars for one participant
%
%

% clear; close all;

% FTI mean, Slopes (s) and their std (i)
L1SES   = struct('f',0.913,'ff',1.206,'s',1e-3*(-9.147),'ss',1e-3*(11.546-6.749)/2);
L1SDSS  = struct('f',1.808,'ff',0.840,'s',1e-3*(-6.796),'ss',1e-3*(8.251-5.340)/2);
% R1SES   = struct('f',1.418,'ff',0.935,'s',1e-3*(-7.223),'ss',(9.021-5.425)/2);
% R1SDSS  = struct('f',2.489,'ff',0.807,'s',1e-3*(-7.539),'ss',(8.566-6.511)/2);

L2SES   = struct('f',1.973,'ff',0.807,'s',1e-3*(-6.094),'ss',1e-3*(7.710-4.477)/2);
L2SDSS  = struct('f',2.956,'ff',0.412,'s',1e-3*(-3.153),'ss',1e-3*(3.959-2.347)/2);
% R2SES   = struct('f',1.457,'ff',0.909,'s',1e-3*(-7.511),'ss',(8.993-6.029)/2);
% R2SDSS  = struct('f',3.202,'ff',0.450,'s',1e-3*(-3.975),'ss',(4.522-3.428)/2);

colors = {...
[     0    0.4470    0.7410];
[0.8500    0.3250    0.0980];
[0.9290    0.6940    0.1250];
[0.4940    0.1840    0.5560];
[0.4660    0.6740    0.1880];
[0.3010    0.7450    0.9330];
[0.6350    0.0780    0.1840]...
};

figure
hold on
plot(L1SES.f, L1SES.s,'o',L1SDSS.f, L1SDSS.s,'v','Color',colors{1},'MarkerFaceColor',colors{1});
plot(L2SES.f, L2SES.s,'o',L2SDSS.f, L2SDSS.s,'v','Color',colors{2},'MarkerFaceColor',colors{2});
xlabel 'FTI mean'
ylabel 'FTI Fitting Slope'
title 'P1 Left Leg'; box on; grid on;
legend('L1SES','L1SDSS','L2SES','L2SDSS','Location','SouthOutside','Orientation','Horizontal');

%% Plot the flawless custom made error bars

Yrange = ylim;
Xrange = xlim;
ErrorCapX = 0.1*(Yrange(2)-Yrange(1));
ErrorCapY = 0.1*(Xrange(2)-Xrange(1));

line([L1SES.f-L1SES.ff L1SES.f+L1SES.ff],[L1SES.s L1SES.s],'Color',colors{1}); % ErrorBarX
line([L1SES.f-L1SES.ff L1SES.f-L1SES.ff],[L1SES.s+ErrorCapX/2 L1SES.s-ErrorCapX/2],'Color',colors{1}); % ErrorCapX
line([L1SES.f+L1SES.ff L1SES.f+L1SES.ff],[L1SES.s+ErrorCapX/2 L1SES.s-ErrorCapX/2],'Color',colors{1}); % ErrorCapX
line([L1SES.f L1SES.f],[L1SES.s-L1SES.ss L1SES.s+L1SES.ss],'Color',colors{1}); % ErrorBarY
line([L1SES.f-ErrorCapY/2 L1SES.f+ErrorCapY/2],[L1SES.s+L1SES.ss L1SES.s+L1SES.ss],'Color',colors{1}); % ErrorCapY
line([L1SES.f-ErrorCapY/2 L1SES.f+ErrorCapY/2],[L1SES.s-L1SES.ss L1SES.s-L1SES.ss],'Color',colors{1}); % ErrorCapY

line([L1SDSS.f-L1SDSS.ff L1SDSS.f+L1SDSS.ff],[L1SDSS.s L1SDSS.s],'Color',colors{1}); % ErrorBarX
line([L1SDSS.f-L1SDSS.ff L1SDSS.f-L1SDSS.ff],[L1SDSS.s+ErrorCapX/2 L1SDSS.s-ErrorCapX/2],'Color',colors{1}); % ErrorCapX
line([L1SDSS.f+L1SDSS.ff L1SDSS.f+L1SDSS.ff],[L1SDSS.s+ErrorCapX/2 L1SDSS.s-ErrorCapX/2],'Color',colors{1}); % ErrorCapX
line([L1SDSS.f L1SDSS.f],[L1SDSS.s-L1SDSS.ss L1SDSS.s+L1SDSS.ss],'Color',colors{1}); % ErrorBarY
line([L1SDSS.f-ErrorCapY/2 L1SDSS.f+ErrorCapY/2],[L1SDSS.s+L1SDSS.ss L1SDSS.s+L1SDSS.ss],'Color',colors{1}); % ErrorCapY
line([L1SDSS.f-ErrorCapY/2 L1SDSS.f+ErrorCapY/2],[L1SDSS.s-L1SDSS.ss L1SDSS.s-L1SDSS.ss],'Color',colors{1}); % ErrorCapY

line([L2SES.f-L2SES.ff L2SES.f+L2SES.ff],[L2SES.s L2SES.s],'Color',colors{2}); % ErrorBarX
line([L2SES.f-L2SES.ff L2SES.f-L2SES.ff],[L2SES.s+ErrorCapX/2 L2SES.s-ErrorCapX/2],'Color',colors{2}); % ErrorCapX
line([L2SES.f+L2SES.ff L2SES.f+L2SES.ff],[L2SES.s+ErrorCapX/2 L2SES.s-ErrorCapX/2],'Color',colors{2}); % ErrorCapX
line([L2SES.f L2SES.f],[L2SES.s-L2SES.ss L2SES.s+L2SES.ss],'Color',colors{2}); % ErrorBarY
line([L2SES.f-ErrorCapY/2 L2SES.f+ErrorCapY/2],[L2SES.s+L2SES.ss L2SES.s+L2SES.ss],'Color',colors{2}); % ErrorCapY
line([L2SES.f-ErrorCapY/2 L2SES.f+ErrorCapY/2],[L2SES.s-L2SES.ss L2SES.s-L2SES.ss],'Color',colors{2}); % ErrorCapY

line([L2SDSS.f-L2SDSS.ff L2SDSS.f+L2SDSS.ff],[L2SDSS.s L2SDSS.s],'Color',colors{2}); % ErrorBarX
line([L2SDSS.f-L2SDSS.ff L2SDSS.f-L2SDSS.ff],[L2SDSS.s+ErrorCapX/2 L2SDSS.s-ErrorCapX/2],'Color',colors{2}); % ErrorCapX
line([L2SDSS.f+L2SDSS.ff L2SDSS.f+L2SDSS.ff],[L2SDSS.s+ErrorCapX/2 L2SDSS.s-ErrorCapX/2],'Color',colors{2}); % ErrorCapX
line([L2SDSS.f L2SDSS.f],[L2SDSS.s-L2SDSS.ss L2SDSS.s+L2SDSS.ss],'Color',colors{2}); % ErrorBarY
line([L2SDSS.f-ErrorCapY/2 L2SDSS.f+ErrorCapY/2],[L2SDSS.s+L2SDSS.ss L2SDSS.s+L2SDSS.ss],'Color',colors{2}); % ErrorCapY
line([L2SDSS.f-ErrorCapY/2 L2SDSS.f+ErrorCapY/2],[L2SDSS.s-L2SDSS.ss L2SDSS.s-L2SDSS.ss],'Color',colors{2}); % ErrorCapY





%% Slopes (s) and their 95% confidence interval (i)
% L1SES   = struct('s',-9.147,'i',(11.546-6.749)/2);
% L1SDSS  = struct('s',-6.796,'i',(8.251-5.340)/2);
% R1SES   = struct('s',-7.223,'i',(9.021-5.425)/2);
% R1SDSS  = struct('s',-7.539,'i',(8.566-6.511)/2);
% 
% L2SES   = struct('s',-6.094,'i',(7.710-4.477)/2);
% L2SDSS  = struct('s',-3.153,'i',(3.959-2.347)/2);
% R2SES   = struct('s',-7.511,'i',(8.993-6.029)/2);
% R2SDSS  = struct('s',-3.975,'i',(4.522-3.428)/2);
% 
% figure
% hold on
% errorbar([2,10],	1e-3*[L1SES.s,R1SES.s],1e-3*[L1SES.i,R1SES.i],      'o', 'Color', [0,0.447,0.741]);
% errorbar([3,11],	1e-3*[L1SDSS.s,R1SDSS.s],1e-3*[L1SDSS.i,R1SDSS.i],  's', 'Color', [0,0.447,0.741]);
% errorbar([5,13],	1e-3*[L2SES.s,R2SES.s],1e-3*[L2SES.i,R2SES.i],      'o', 'Color', [0.85,0.325,0.098]);
% errorbar([6,14],	1e-3*[L2SDSS.s,R2SDSS.s],1e-3*[L2SDSS.i,R2SDSS.i],  's', 'Color', [0.85,0.325,0.098]);
% plot([8 8],ylim,'Color',[0.15,0.15,0.15,0.15])
% xlim([0 16]); title 'P1'; box on;
% legend('SES 1','SDSS 1','SES 2','SDSS 2','Location','SouthOutside','Orientation','Horizontal');
% set(gca,'XTick',[4 12],'XTickLabel',{'Left Leg','Right Leg'},'TickLength',[0 0],'Ygrid', 'on');
% hold off
