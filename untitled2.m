% Find start and end indices of all zero sequences
zsig = ~~StimCurrentTrim.Data; % 0 for zeros, 1 for everyhting else
dsig = diff([1;zsig;1]); % 1 for rising edge, -1 for falling
SIndexVec = find(dsig < 0); % Vector w/ start indices
EIndexVec = find(dsig > 0)-1; % Vector w/ end indices
ValidVec = ((EIndexVec-SIndexVec) >= 1); % Get rid of small peaks
SIndexVec = SIndexVec(ValidVec); % Valid start indices
EIndexVec = EIndexVec(ValidVec); % Valid end indices
SortedTimes = StimCurrentTrim.Time(sort([SIndexVec;EIndexVec]));
CompareM = pdist2(ForceTrim.Time,SortedTimes); % Compares each element
[~,temp] = min(CompareM); % Get index of closest time
IndexM = vec2mat(temp,2); % Column1 is start, column2 is end
% Get the offset from start-end then apply it to the data
IndexM(end+1,1) = IndexM(end,2)+1; % Dealing with the last zero sequence
H = zeros(length(ForceTrim.Data),1); % Pre-allocating
for i=1:size(IndexM,1)-1
    idxA10 = IndexM(i,1) + ceil(0.1*(IndexM(i,2)-IndexM(i,1)));
    Foffset = mean(ForceTrim.Data(idxA10:IndexM(i,2)));
    H(IndexM(i,1):(IndexM(i+1,1)-1)) = ...
        ForceTrim.Data(IndexM(i,1):(IndexM(i+1,1)-1)) - Foffset;
end
