%%
g = 9.8067;

ExcludeIdx = [];
MidStimTimes = FileStruct.E1_SUBX0_1M_Left.MidStimTimes;
AreasVec = FileStruct.E1_SUBX0_1M_Left.AreasVec;
ForceNorm = FileStruct.E1_SUBX0_1M_Left.ForceNorm;

AreasVec = g*AreasVec;
ForceNorm.Data = g*ForceNorm.Data;

[CurveFitData,CurveFitStats,~] = fit(MidStimTimes,AreasVec,'poly1',...
    'Exclude', ExcludeIdx);

%%
figure
hold on
yyaxis left % Left Y axis 
plot(ForceNorm)
ylabel('Force (N)'), xlim([-10 380])
title(' ')
LimitsL = [-1.9 12]; ylim(LimitsL); box on;
yyaxis right  % Right Y axis 

if ~isempty(ExcludeIdx)
    ExcludeVec = false(1,length(AreasVec));
    ExcludeVec(ExcludeIdx) = true;
    plot(CurveFitData,MidStimTimes,AreasVec,'o',ExcludeVec,'x')
    legend('Force','FTI','Excluded','FTI Fit','Location','Best')
else
    plot(CurveFitData,MidStimTimes,AreasVec,'o')
    legend('Force','FTI','FTI Fit','Location','Best')
end

LimitsR = [0 40];
ylim([LimitsL(1)*LimitsR(2)/LimitsL(2) LimitsR(2)])
ylabel('Force-Time Integral (N.s)'), xlabel('Time (s)')
hold off