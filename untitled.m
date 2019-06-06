% Find consecutive zeros in a signal
zsig = ~~StimCurrentTrim1.Data; % 0 for zeros, 1 for everyhting else
dsig = diff([1;zsig;1]); % 1 for rising edge, -1 for falling
SIndexVec = find(dsig < 0); % Vector w/ start indices
EIndexVec = find(dsig > 0)-1; % Vector w/ end indices
DurationVec = EIndexVec-SIndexVec+1; % Number of zeros in-between
ValidVec = (DurationVec >= 2); % Get rid of small peaks
SIndexVec = SIndexVec(ValidVec); % Valid start indices
EIndexVec = EIndexVec(ValidVec); % Valid end indices
IndexM = [SIndexVec, EIndexVec];

IndexM(end+1,:) = [IndexM(end,2),IndexM(end,2)];
TimeM = [StimCurrentTrim1.Time(IndexM(:,1)),StimCurrentTrim1.Time(IndexM(:,2))];
for i=1:length(IndexM)-1
    indexA = find(ForceTrim1.Time>STimeVec(i),1);
    indexB = find(ForceTrim1.Time<ETimeVec(i),1,'last');
    m = mean(ForceTrim1.Data(indexA:indexB));
    ForceTrim1.Data(IndexM(i,1):IndexM(i+1,1)) = ForceTrim1.Data(IndexM(i,1):IndexM(i+1,1)) - m;
    v = IndexM(i,1):IndexM(i,2);
end
