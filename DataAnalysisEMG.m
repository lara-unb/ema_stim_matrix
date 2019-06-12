%
%
% MATLAB code - EMA Matrix Experiments
% 2019-06-11
% Lucas de Macedo Pinheiro
% 
%   Parse the EMG files and .............................
%

FilesEMG  = dir('Matrix*EMG_*.mat');
FilesEMGS = dir('Matrix*EMGS*.mat');
RawData = load(FilesEMG(1).name);
RawData = RawData.ans;
RawStatus = load(FilesEMGS(1).name);
RawStatus = RawStatus.ans;



% FullRawSignal 