% Load files
load('MATLAB_Matrix_Pilot_1M_Left_S1.mat');
ForceTrim1 = Force;
StimCurrentTrim1 = StimCurrent;
StimCommandData1 = StimCommand;
load('MATLAB_Matrix_Pilot_1M_Left_S2.mat');
ForceTrim2 = Force;
StimCurrentTrim2 = StimCurrent;
StimCommandData2 = StimCommand;
load('MATLAB_Matrix_Pilot_1M_Left_S3.mat');
ForceTrim3 = Force;
StimCurrentTrim3 = StimCurrent;
StimCommandData3 = StimCommand;

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

% Saves data
save('MATLAB_Matrix_Pilot_1M_Left','Force','StimCurrent','StimCommand')
