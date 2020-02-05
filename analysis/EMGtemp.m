%%
EMGdata = timeseries(EMGdata,t);
clear t
plot(EMGdata)
EMGinitial = EMGdata;

%%
w=8;
ind = find(EMGdata.Data>0.22);
EMGdata.Data(ind) = EMGdata.Data(ind-1);
clear ind

FileStruct.(Fields{w}).EMGdata = EMGdata;

%%
k=8; 
g = FileStruct.(Fields{k});
figure
plot(g.EMGdata)
hold on
plot(g.StimCurrent.Time, g.StimCurrent.Data./400)

%%
ti = g.StimCommandZeroed.Time(1)-5; % 5s before stim starts
tf = g.StimCommandZeroed.Time(end)+5; % 5s after stim ends
% Maintaining Time Series
FileStruct.(Fields{k}).EMGdata = getsamples(g.EMGdata,...
    find(g.EMGdata.Time>ti,1):find(g.EMGdata.Time<tf,1,'last'));

EMGdata = FileStruct.(Fields{w}).EMGdata;

%%
Original = EMGdata;
FileStruct.(Fields{k}).EMGinitial = EMGdata;
StimCommandZeroed = FileStruct.(Fields{w}).StimCommandZeroed;
StimCurrentTrim = g.StimCurrentTrim;

%% OFFSET

FileStruct.(Fields{k}).EMGNoTrend = NoTrend;
FileStruct.(Fields{k}).EMGdata = NoTrend;
EMGdata = FileStruct.(Fields{w}).EMGdata;

%%
EMGExcludeIdx = [];
StimCommandZeroed = FileStruct.(Fields{w}).StimCommandZeroed;







%%
a = find(EMGdata.Time<5.52,1,'last');
EMGdata.Time(find(EMGdata.Time<5.52,1,'last'))
b = find(EMGdata.Time<16.8,1,'last');
EMGdata.Time(find(EMGdata.Time<16.8,1,'last'))
ab = b-a-1;
EMGdata = delsample(EMGdata,'Index',1:ab);
EMGdata.Time = EMGdata.Time - EMGdata.Time(1);
plot(EMGdata)


emg = EMGdata;
FileStruct.E1_SUBX0_1C_Left.EMGdata = EMGdata;
bkp = FileStruct;



%%
w=3;
EMGdata = FileStruct.(Fields{w}).EMGdata;
StimCommandZeroed = FileStruct.(Fields{w}).StimCommandZeroed;

%%
FileStruct.(Fields{k}).EMGinitial = g.EMGdata;
FileStruct.(Fields{k}).EMGNoTrend = NoTrend;
FileStruct.(Fields{k}).EMGdata = NoTrend;


for w = 2:8
    
    g = FileStruct.(Files{w});
       
    figure
    hold on
    plot(g.EMGNorm)
    plot(g.ForceNorm)
    ylabel('Amplitude'), xlim([-10 380])
    title(Files{w},'Interpreter', 'none')
    LimitsL = [-.19 2]; ylim(LimitsL); box on;
    legend('eEMG','Force','Location','Best')
    xlabel('Time (s)')
    hold off
    
end

