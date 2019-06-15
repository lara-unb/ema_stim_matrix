%
%
% MATLAB code - EMA Matrix Experiments
% 2019-06-11
% Lucas de Macedo Pinheiro
% 
%   Parse the EMG files and .............................
%

% Load the Signal and Status files
FilesEMG  = dir('Matrix*EMG_*.mat');
FilesEMGS = dir('Matrix*EMGS*.mat');
RawData = load(FilesEMG(1).name);
RawData = RawData.ans;
RawStatus = load(FilesEMGS(1).name);
RawStatus = RawStatus.ans;

% Rearrange the data into single column
MaxStatus = max(RawStatus(3,:));
ValidDataIdxVec = find(RawStatus(3,:));
ValidData = RawData(1:MaxStatus+1,ValidDataIdxVec);

h = ValidData(2:end,:);
temp = h(h~=-1);

% Assign individual timestamp to data


% Remove 60 Hz influence from powerline
d = designfilt('bandstopiir','FilterOrder',2, ...
               'HalfPowerFrequency1',59,'HalfPowerFrequency2',61, ...
               'DesignMethod','butter','SampleRate',4000);
           
[popen,fopen] = periodogram(temp,[],[],4000);
[pbutt,fbutt] = periodogram(buttLoop,[],[],4000);

plot(fopen,20*log10(abs(popen)),fbutt,20*log10(abs(pbutt)),'--')
ylabel 'Power/frequency (dB/Hz)', xlabel 'Frequency (Hz)'
title 'Power Spectrum', legend('Unfiltered','Filtered')

% FullRawSignal 