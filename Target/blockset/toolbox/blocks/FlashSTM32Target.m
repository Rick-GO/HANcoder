%%***************************************************************************************
%% file         FlashSTM32Target.m
%% brief        Allows calling the flashtool automatically.
%%
%%---------------------------------------------------------------------------------------
%%                          C O P Y R I G H T
%%---------------------------------------------------------------------------------------
%%  Copyright 2019 (c) by HAN Automotive     http://www.han.nl     All rights reserved
%%
%%---------------------------------------------------------------------------------------
%%                            L I C E N S E
%%---------------------------------------------------------------------------------------
%% Permission is hereby granted, free of charge, to any person obtaining a copy of this
%% software and associated documentation files (the "Software"), to deal in the Software
%% without restriction, including without limitation the rights to use, copy, modify, merge,
%% publish, distribute, sublicense, and/or sell copies of the Software, and to permit
%% persons to whom the Software is furnished to do so, subject to the following conditions:
%%
%% The above copyright notice and this permission notice shall be included in all copies or
%% substantial portions of the Software.
%%
%% THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
%% INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
%% PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE
%% FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR
%% OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
%% DEALINGS IN THE SOFTWARE.
%%
%%***************************************************************************************
%% Automatic flash script
function FlashSTM32Target(modelName)
% find settings block
HANcoderBlocks = find_system(modelName, 'RegExp', 'on', 'LookUnderMasks', 'All', 'Tag', 'HANcoder_TARGET_.');
for index=1:1:length(HANcoderBlocks) % Loop through the HANcoder blocks
    HANcoderStruct = get_param(HANcoderBlocks{index},'UserData');
    BlockType = HANcoderStruct.BlockType;
    if strcmp(BlockType,'Settings')
        MaskValues = get_param(HANcoderBlocks{index},'MaskValues');
        if ~strcmp(MaskValues{2},'off')
            % Check if the HANcoder_BOOTLOADER setting is correct
            MakeCommand = get_param(bdroot(modelName),'MakeCommand');
            if ~isempty(strfind(MakeCommand, 'HANcoder_BOOTLOADER=0'))
                warndlg(['The model was not built for flashing with the bootloader. Flash procedure will not be started.',...
				'Please set HANcoder_BOATLOADER=1 in Configuration Options->Code Generation->Make command, if you wish to',...
				' use the automatic flash function'],'HANcoder warning');
            else
                disp('### Starting automatic flash procedure');
                FlashViaMicroBoot(modelName,MaskValues{2});
            end
            break
        end
    end
end
end % end of function FlashSTM32Target()

function FlashViaMicroBoot(modelName,Interface)
ModelPath= strcat(' "',pwd, '\..\',bdroot(modelName),'.srec"');
MircoBootPath = strcat('"',pwd,'\..\..\Host\MicroBoot\MicroBoot.exe"');
CubeProgrammerPath = strcat('"',pwd,'\..\..\Host\CubeProgrammer\STM32_Programmer_CLI.exe"');
switch Interface
    case 'USB'
        cmdCommand = strcat(MircoBootPath, ' -iopenblt_usb.dll ', ModelPath, ' &');
	case 'USB-F3'
		cmdCommand = strcat(CubeProgrammerPath, ' -c port=usb1 -e all -w ', ModelPath, ' --go 0x08000000');
    case 'CAN'
        cmdCommand = strcat(MircoBootPath, ' -iopenblt_can_peak.dll ', ModelPath, ' &');
    case 'Ethernet'
        cmdCommand = strcat(MircoBootPath, ' -iopenblt_net.dll ', ModelPath, ' &');
end
status = system(cmdCommand);
  % In case upload to Olimexino F3 target didn't succeed
  if(status == 1 && strcmp(Interface,'USB-F3'))
  FlashTargetRetry(CubeProgrammerPath,ModelPath);
  end
end % end of function FlashSTM32Target()

%% A function that ask user if the target is connected and in bootloader mode
function FlashTargetRetry(CubeProgrammerPath,ModelPath)
action = questdlg({'Olimexino F3 not found. Is your target connected and in bootloader mode?',...
'Connect Olimexino F3 and bring target in bootloader mode and retry.'},'HANcoder upload error','Retry','Cancel','Retry');
  if (strcmp(action,'Retry'))
  cmdCommand = strcat(CubeProgrammerPath, ' -c port=usb1 -e all -w ', ModelPath, ' --go 0x08000000');
  system(cmdCommand);
  else
  disp('### Automatic flash procedure canceled by user');
  end
end % end of function FlashTargetRetry()
%% end of file