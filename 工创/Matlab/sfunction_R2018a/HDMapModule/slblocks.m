function blkStruct = slblocks
%Function to add a specific custom library to the Library Browser 

% Define how the custom library is displayed in the Library Browser 
Browser(1).Library = 'HDMapModule'; %库文件名称
Browser(1).Name    = 'HDMapModule'; %在库浏览器中显示的名称
Browser(1).IsFlat  = 1; %该库文件是否有子系统

% The function that will be called when the user double-clicks on
% this icon.
blkStruct.Name = ['HDMapModule'];
blkStruct.OpenFcn = 'HDMapModule';
blkStruct.MaskDisplay = ''; 

blkStruct.Browser = Browser;

% End of blocks



