function blkStruct = slblocks
%Function to add a specific custom library to the Library Browser 

% Define how the custom library is displayed in the Library Browser 
Browser(1).Library = 'SimOneModule'; %库文件名称
Browser(1).Name    = 'SimOneModule'; %在库浏览器中显示的名称
Browser(1).IsFlat  = 1; %该库文件是否有子系统

% The function that will be called when the user double-clicks on
% this icon.
blkStruct.Name = ['SimOneModule'];
blkStruct.OpenFcn = 'SimOneModule';
blkStruct.MaskDisplay = ''; 

blkStruct.Browser = Browser;

% End of blocks



