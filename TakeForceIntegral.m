%
% MATLAB code - EMA Matrix Experiments
% 2019-07-04
% Lucas de Macedo Pinheiro
% 
%   Integrate the Force x Time graph.
%
%

StimEdges = find(diff(StimCommandZeroed.Time)>4); % Edge threshold
StimEdges = sort([StimEdges; StimEdges+1]); % Both rising and falling edges
StimEdges = StimCommandZeroed.Time([1; StimEdges; end]); % Get time

CompareM = pdist2(Force.Time,StimEdges); % Compares each element
[~,AllIndicesVec] = min(CompareM); % Get index of closest time
IndexM = vec2mat(AllIndicesVec,2); % Column1 is start, column2 is end

ForceStimCell = cell(length(IndexM),1); % Cell with the 36 timeXforce data
AreasVec = zeros(length(IndexM),1);
for i=1:length(IndexM)
    IdxVec = IndexM(i,1):IndexM(i,2);
    ForceStimCell{i,1} = [Force.Time(IdxVec) Force.Data(IdxVec)];
    AreasVec(i,1) = trapz(ForceStimCell{i}(:,1),ForceStimCell{i,1}(:,2));
end


figure
hold on

yyaxis left
plot(Force)
xlim([-10 380])
LimitsL = ylim;

yyaxis right
MidForceStim = Force.Time(round(mean(IndexM,2)));
plot(MidForceStim,AreasVec,'o')
LimitsR = ylim;
ylim([LimitsL(1)*LimitsR(2)/LimitsL(2) LimitsR(2)])

hold off




