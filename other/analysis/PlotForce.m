%%
ExcludeIdx = [];
MidStimTimes = FileStruct.E1_SUBX0_1C_Left.MidStimTimes;
AreasVec = FileStruct.E1_SUBX0_1C_Left.AreasVec;
ForceNorm = FileStruct.E1_SUBX0_1C_Left.ForceNorm;
CurveFitData = FileStruct.E1_SUBX0_1C_Left.CurveFitData;
CurveFitStats = FileStruct.E1_SUBX0_1C_Left.CurveFitStats;

%%
figure
hold on
yyaxis left % Left Y axis 
plot(ForceNorm)
ylabel('Normalized Force'), xlim([-10 380])
title(' ')
LimitsL = [-0.09 1.2]; ylim(LimitsL); box on;
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

LimitsR = [0 4];
ylim([LimitsL(1)*LimitsR(2)/LimitsL(2) LimitsR(2)])
ylabel('Normalized Force ? Time (FTI)'), xlabel('Time (s)')
hold off