% Load files

% Uncomment these for each leg
% fprintf('\n\nRIGHT LEG DATA\n');
% Files = dir('MATLAB*Right*.mat');
fprintf('\n\nLEFT LEG DATA\n');
Files = dir('MATLAB*Left*.mat');

for w = 1:length(Files)
    load(Files(w).name);
    if w == 1
        ForceTrim1 = Force;
        StimCurrentTrim1 = StimCurrent;
        StimCommandData1 = StimCommand;
    elseif w == 2
        ForceTrim2 = Force;
        StimCurrentTrim2 = StimCurrent;
        StimCommandData2 = StimCommand;
    elseif w == 3
        ForceTrim3 = Force;
        StimCurrentTrim3 = StimCurrent;
        StimCommandData3 = StimCommand;
    end
end

% Trim data at the end to concatenate 
tf = StimCommandData1.Time(end);
Force2NewTi = ForceTrim1.Time(find(ForceTrim1.Time>tf,1));
ForceTrim1 = getsamples(ForceTrim1,1:find(ForceTrim1.Time==Force2NewTi)-1);
StimCurr2NewTi = StimCurrentTrim1.Time(find(StimCurrentTrim1.Time>tf,1));
StimCurrentTrim1 = getsamples(StimCurrentTrim1,1:find(StimCurrentTrim1.Time==StimCurr2NewTi)-1);
StimCom2NewTi = StimCommandData1.Time(end);

Force = ForceTrim2;
Force.Time = ForceTrim2.Time + Force2NewTi;
StimCurrent = StimCurrentTrim2;
StimCurrent.Time = StimCurrentTrim2.Time + StimCurr2NewTi;
StimCommand = StimCommandData2;
StimCommand.Time = StimCommandData2.Time + StimCom2NewTi;

Force = append(ForceTrim1, Force);
StimCurrent = append(StimCurrentTrim1,StimCurrent);
StimCommand = [StimCommandData1;StimCommand];

StimCommandData1 = StimCommand;
ForceTrim1 = Force;
StimCurrentTrim1 = StimCurrent;
ForceTrim2 = ForceTrim3;
StimCurrentTrim2 = StimCurrentTrim3;
StimCommandData2 = StimCommandData3;

tf = StimCommandData1.Time(end);
Force2NewTi = ForceTrim1.Time(find(ForceTrim1.Time>tf,1));
ForceTrim1 = getsamples(ForceTrim1,1:find(ForceTrim1.Time==Force2NewTi)-1);
StimCurr2NewTi = StimCurrentTrim1.Time(find(StimCurrentTrim1.Time>tf,1));
StimCurrentTrim1 = getsamples(StimCurrentTrim1,1:find(StimCurrentTrim1.Time==StimCurr2NewTi)-1);
StimCom2NewTi = StimCommandData1.Time(end);

Force = ForceTrim2;
Force.Time = ForceTrim2.Time + Force2NewTi;
StimCurrent = StimCurrentTrim2;
StimCurrent.Time = StimCurrentTrim2.Time + StimCurr2NewTi;
StimCommand = StimCommandData2;
StimCommand.Time = StimCommandData2.Time + StimCom2NewTi;

Force = append(ForceTrim1, Force);
StimCurrent = append(StimCurrentTrim1,StimCurrent);
StimCommand = [StimCommandData1;StimCommand];

% Plots final data
figure
plot(Force)
hold on
plot(StimCurrent./40)
title(Files(1).name(1:end-7))
xlabel('Time (s)')
xlim([-10 380])
legend('Force (kg)','Stimulation (Y/N)')

% Saves data
save(Files(1).name(1:end-7),'Force','StimCurrent','StimCommand')
