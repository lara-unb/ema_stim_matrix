%
% MATLAB code - EMA Matrix Experiments
% 2020-01-25
% Lucas de Macedo Pinheiro
% 
%   Plot the slope and errorbars for one participant
%
%

clear; close all;

% Open window for file selection
disp('Select the MAT files...');
Files = uigetfile('*.mat','Select The MAT Files','MultiSelect','on');
if isa(Files,'char') % Only one file selected
   Files = {Files}; 
end

%% Extract data from struct
load(Files{1});
Fields = fieldnames(FileStruct);
    
for k = 1:numel(Fields)
    g = FileStruct.(Fields{k});
    c = g.LinearModelFit.Coefficients;
    name = Fields{k};
    
    % FTI mean, Slopes (s) and their std (i)
    if strcmp(name(1:2),'E1')
        if strcmp(name(11:end),'C_Left')
            L1SES  = struct('f',g.FTI.mean,'ff',g.FTI.std,'s',c.Estimate(2),'ss',c.SE(2));
        elseif strcmp(name(11:end),'M_Left')
            L1SDSS = struct('f',g.FTI.mean,'ff',g.FTI.std,'s',c.Estimate(2),'ss',c.SE(2));
        elseif strcmp(name(11:end),'C_Right')
            R1SES  = struct('f',g.FTI.mean,'ff',g.FTI.std,'s',c.Estimate(2),'ss',c.SE(2));
        elseif strcmp(name(11:end),'M_Right')
            R1SDSS = struct('f',g.FTI.mean,'ff',g.FTI.std,'s',c.Estimate(2),'ss',c.SE(2));
        end
    else
        if strcmp(name(11:end),'C_Left')
            L2SES  = struct('f',g.FTI.mean,'ff',g.FTI.std,'s',c.Estimate(2),'ss',c.SE(2));
        elseif strcmp(name(11:end),'M_Left')
            L2SDSS = struct('f',g.FTI.mean,'ff',g.FTI.std,'s',c.Estimate(2),'ss',c.SE(2));
        elseif strcmp(name(11:end),'C_Right')
            R2SES  = struct('f',g.FTI.mean,'ff',g.FTI.std,'s',c.Estimate(2),'ss',c.SE(2));
        elseif strcmp(name(11:end),'M_Right')
            R2SDSS = struct('f',g.FTI.mean,'ff',g.FTI.std,'s',c.Estimate(2),'ss',c.SE(2));
        end
    end
end

%% Plot the center points for LEFT leg
colors = {...
[172   30   65]/255;
[240  161   19]/255;
[  5  177  199]/255;
[168  186    5]/255;...
};

figure
hold on
plot(L1SES.f, L1SES.s,'o',L1SDSS.f, L1SDSS.s,'v','Color',colors{1},'MarkerFaceColor',colors{1});
plot(L2SES.f, L2SES.s,'o',L2SDSS.f, L2SDSS.s,'v','Color',colors{2},'MarkerFaceColor',colors{2});
xlabel 'FTI mean'
ylabel 'FTI Fitting Slope'
title(['P' num2str(str2double(Fields{1}(8))+1) ' Left Leg']); box on; grid on;
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

hold off

%%
disp('Saving figure and file...');
savefig(['P' num2str(str2double(Fields{1}(8))+1) 'LeftLeg.fig']);

%% Plot the center points for RIGHT leg
colors = {...
[  5  177  199]/255;
[168  186    5]/255;
[172   30   65]/255;
[240  161   19]/255;...
};

figure
hold on
plot(R1SES.f, R1SES.s,'o',R1SDSS.f, R1SDSS.s,'v','Color',colors{1},'MarkerFaceColor',colors{1});
plot(R2SES.f, R2SES.s,'o',R2SDSS.f, R2SDSS.s,'v','Color',colors{2},'MarkerFaceColor',colors{2});
xlabel 'FTI mean'
ylabel 'FTI Fitting Slope'
title(['P' num2str(str2double(Fields{1}(8))+1) ' Right Leg']); box on; grid on;
legend('R1SES','R1SDSS','R2SES','R2SDSS','Location','SouthOutside','Orientation','Horizontal');

%% Plot the flawless custom made error bars

Yrange = ylim;
Xrange = xlim;
ErrorCapX = 0.1*(Yrange(2)-Yrange(1));
ErrorCapY = 0.1*(Xrange(2)-Xrange(1));

line([R1SES.f-R1SES.ff R1SES.f+R1SES.ff],[R1SES.s R1SES.s],'Color',colors{1}); % ErrorBarX
line([R1SES.f-R1SES.ff R1SES.f-R1SES.ff],[R1SES.s+ErrorCapX/2 R1SES.s-ErrorCapX/2],'Color',colors{1}); % ErrorCapX
line([R1SES.f+R1SES.ff R1SES.f+R1SES.ff],[R1SES.s+ErrorCapX/2 R1SES.s-ErrorCapX/2],'Color',colors{1}); % ErrorCapX
line([R1SES.f R1SES.f],[R1SES.s-R1SES.ss R1SES.s+R1SES.ss],'Color',colors{1}); % ErrorBarY
line([R1SES.f-ErrorCapY/2 R1SES.f+ErrorCapY/2],[R1SES.s+R1SES.ss R1SES.s+R1SES.ss],'Color',colors{1}); % ErrorCapY
line([R1SES.f-ErrorCapY/2 R1SES.f+ErrorCapY/2],[R1SES.s-R1SES.ss R1SES.s-R1SES.ss],'Color',colors{1}); % ErrorCapY

line([R1SDSS.f-R1SDSS.ff R1SDSS.f+R1SDSS.ff],[R1SDSS.s R1SDSS.s],'Color',colors{1}); % ErrorBarX
line([R1SDSS.f-R1SDSS.ff R1SDSS.f-R1SDSS.ff],[R1SDSS.s+ErrorCapX/2 R1SDSS.s-ErrorCapX/2],'Color',colors{1}); % ErrorCapX
line([R1SDSS.f+R1SDSS.ff R1SDSS.f+R1SDSS.ff],[R1SDSS.s+ErrorCapX/2 R1SDSS.s-ErrorCapX/2],'Color',colors{1}); % ErrorCapX
line([R1SDSS.f R1SDSS.f],[R1SDSS.s-R1SDSS.ss R1SDSS.s+R1SDSS.ss],'Color',colors{1}); % ErrorBarY
line([R1SDSS.f-ErrorCapY/2 R1SDSS.f+ErrorCapY/2],[R1SDSS.s+R1SDSS.ss R1SDSS.s+R1SDSS.ss],'Color',colors{1}); % ErrorCapY
line([R1SDSS.f-ErrorCapY/2 R1SDSS.f+ErrorCapY/2],[R1SDSS.s-R1SDSS.ss R1SDSS.s-R1SDSS.ss],'Color',colors{1}); % ErrorCapY

line([R2SES.f-R2SES.ff R2SES.f+R2SES.ff],[R2SES.s R2SES.s],'Color',colors{2}); % ErrorBarX
line([R2SES.f-R2SES.ff R2SES.f-R2SES.ff],[R2SES.s+ErrorCapX/2 R2SES.s-ErrorCapX/2],'Color',colors{2}); % ErrorCapX
line([R2SES.f+R2SES.ff R2SES.f+R2SES.ff],[R2SES.s+ErrorCapX/2 R2SES.s-ErrorCapX/2],'Color',colors{2}); % ErrorCapX
line([R2SES.f R2SES.f],[R2SES.s-R2SES.ss R2SES.s+R2SES.ss],'Color',colors{2}); % ErrorBarY
line([R2SES.f-ErrorCapY/2 R2SES.f+ErrorCapY/2],[R2SES.s+R2SES.ss R2SES.s+R2SES.ss],'Color',colors{2}); % ErrorCapY
line([R2SES.f-ErrorCapY/2 R2SES.f+ErrorCapY/2],[R2SES.s-R2SES.ss R2SES.s-R2SES.ss],'Color',colors{2}); % ErrorCapY

line([R2SDSS.f-R2SDSS.ff R2SDSS.f+R2SDSS.ff],[R2SDSS.s R2SDSS.s],'Color',colors{2}); % ErrorBarX
line([R2SDSS.f-R2SDSS.ff R2SDSS.f-R2SDSS.ff],[R2SDSS.s+ErrorCapX/2 R2SDSS.s-ErrorCapX/2],'Color',colors{2}); % ErrorCapX
line([R2SDSS.f+R2SDSS.ff R2SDSS.f+R2SDSS.ff],[R2SDSS.s+ErrorCapX/2 R2SDSS.s-ErrorCapX/2],'Color',colors{2}); % ErrorCapX
line([R2SDSS.f R2SDSS.f],[R2SDSS.s-R2SDSS.ss R2SDSS.s+R2SDSS.ss],'Color',colors{2}); % ErrorBarY
line([R2SDSS.f-ErrorCapY/2 R2SDSS.f+ErrorCapY/2],[R2SDSS.s+R2SDSS.ss R2SDSS.s+R2SDSS.ss],'Color',colors{2}); % ErrorCapY
line([R2SDSS.f-ErrorCapY/2 R2SDSS.f+ErrorCapY/2],[R2SDSS.s-R2SDSS.ss R2SDSS.s-R2SDSS.ss],'Color',colors{2}); % ErrorCapY

hold off

%%
disp('Saving figure and file...');
savefig(['P' num2str(str2double(Fields{1}(8))+1) 'RightLeg.fig']);

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
