% Load files
load('MATLAB_Matrix_SUB01_2M_Left_S1.mat');
ForceTrim1 = ForceTrim;
StimCurrentTrim1 = StimCurrentTrim;
StimCommandData1 = StimCommandData;
load('MATLAB_Matrix_SUB01_2M_Left_S2.mat');
ForceTrim2 = ForceTrim;
StimCurrentTrim2 = StimCurrentTrim;
StimCommandData2 = StimCommandData;
load('MATLAB_Matrix_SUB01_2M_Left_S3.mat');
ForceTrim3 = ForceTrim;
StimCurrentTrim3 = StimCurrentTrim;
StimCommandData3 = StimCommandData;


% Adjust the time to begin with zero
Toffset = min([ForceTrim1.Time(1),StimCurrentTrim1.Time(1)]);
ForceTrim1.Time = ForceTrim1.Time - Toffset;
StimCurrentTrim1.Time = StimCurrentTrim1.Time - Toffset;
StimCommandData1.Time = StimCommandData1.Time - Toffset;

Toffset = min([ForceTrim2.Time(1),StimCurrentTrim2.Time(1)]);
ForceTrim2.Time = ForceTrim2.Time - Toffset;
StimCurrentTrim2.Time = StimCurrentTrim2.Time - Toffset;
StimCommandData2.Time = StimCommandData2.Time - Toffset;

Toffset = min([ForceTrim3.Time(1),StimCurrentTrim3.Time(1)]);
ForceTrim3.Time = ForceTrim3.Time - Toffset;
StimCurrentTrim3.Time = StimCurrentTrim3.Time - Toffset;
StimCommandData3.Time = StimCommandData3.Time - Toffset;

% Get rid of trending data with envelope function
ForceTrim1bkp = ForceTrim1;
ForceTrim2bkp = ForceTrim2;
ForceTrim3bkp = ForceTrim3;

[~,LPeakFit] = envelope(ForceTrim1.Data,80,'peak');
ForceTrim1.Data = ForceTrim1.Data - LPeakFit;

[~,LPeakFit] = envelope(ForceTrim2.Data,80,'peak');
ForceTrim2.Data = ForceTrim2.Data - LPeakFit;

[~,LPeakFit] = envelope(ForceTrim3.Data,90,'peak');
ForceTrim3.Data = ForceTrim3.Data - LPeakFit;

% plot(ForceTrim3bkp)
% hold on
% plot(ForceTrim3bkp.Time, LPeakFit)
% plot(ForceTrim3bkp.Time,ForceTrim3bkp.Data - LPeakFit)

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
% plot(StimCommand.Time,1.5*ones(1,length(StimCommand.Time)),'.')

% Saves data
save('MATLAB_Matrix_SUB01_2M_Left','Force','StimCurrent','StimCommand')
