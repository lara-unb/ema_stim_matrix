%
% MATLAB code - EMA Matrix Experiments
% 2019-07-04
% Lucas de Macedo Pinheiro
% 
%   Integrate the Force x Time graph and find fitting line.
%
%

clear; close all;

% Open window for file selection
disp('Select the MAT files...');
Files = uigetfile('*.mat','Select The MAT Files','MultiSelect','on');
if isa(Files,'char') % Only one file selected
   Files = {Files}; 
end

for w = 1:length(Files)
% Import MAT file
    fprintf('\n\nLoading "%s" file...\n', Files{w});
    load(Files{w});
    
% Calculate the force integral during stimulation sequences
    disp('Calculating areas...');
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

% Perform data fitting w/ poly1 (a*x + b)
    disp('Fitting line...');
    MidStimTimes = Force.Time(round(mean(IndexM,2)));
    [CurveFitData,~,~] = fit(MidStimTimes,AreasVec,'poly1');
    CurveFitCoefs = coeffvalues(CurveFitData); % Get a and b
    CurveFitEq = sprintf('%.4ft %+ .4f',CurveFitCoefs);
    disp(CurveFitData);

    figure
    hold on
    yyaxis left % Left Y axis 
    plot(Force)
    ylabel('Force (kgf)'), xlim([-10 380])
    title(Files{w}(15:end-4),'Interpreter', 'none')
    LimitsL = ylim;
    yyaxis right  % Right Y axis 
    plot(MidStimTimes,AreasVec,'o')
    plot(CurveFitData)
    LimitsR = ylim;
    ylim([LimitsL(1)*LimitsR(2)/LimitsL(2) LimitsR(2)])
    ylabel('Area (kgf.s)'), xlabel('Time (s)')
    legend('Force','Area',CurveFitEq)
    hold off
    
    disp('Saving figure and file...');
    savefig([Files{w}(15:end-4) '_Fitted.fig']);
end


