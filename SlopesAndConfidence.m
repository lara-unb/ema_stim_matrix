
% Slopes (s) and their 95% confidence interval (i)
L1SES   = struct('s',-9.147,'i',(11.546-6.749)/2);
L1SDSS  = struct('s',-6.796,'i',(8.251-5.340)/2);
R1SES   = struct('s',-7.223,'i',(9.021-5.425)/2);
R1SDSS  = struct('s',-7.539,'i',(8.566-6.511)/2);

L2SES   = struct('s',-6.094,'i',(7.710-4.477)/2);
L2SDSS  = struct('s',-3.153,'i',(3.959-2.347)/2);
R2SES   = struct('s',-7.511,'i',(8.993-6.029)/2);
R2SDSS  = struct('s',-3.975,'i',(4.522-3.428)/2);

figure
hold on
errorbar([2,10],	[L1SES.s,R1SES.s],[L1SES.i,R1SES.i],      'o', 'Color', [0,0.447,0.741]);
errorbar([3,11],	[L1SDSS.s,R1SDSS.s],[L1SDSS.i,R1SDSS.i],  's', 'Color', [0,0.447,0.741]);
errorbar([4,12],	[L2SES.s,R2SES.s],[L2SES.i,R2SES.i],      'o', 'Color', [0.85,0.325,0.098]);
errorbar([5,13],	[L2SDSS.s,R2SDSS.s],[L2SDSS.i,R2SDSS.i],  's', 'Color', [0.85,0.325,0.098]);
plot([5.5 5.5],ylim,'Color',[0.15,0.15,0.15,0.15])
xlim([0 15])
legend('SES 1','SDSS 1'); title 'First Set'; box 'on';
set(gca,'XTick',[2.5 8.5],'XTickLabel',{'Left Leg','Right Leg'},'TickLength',[0 0], 'Ygrid', 'on');
hold off





