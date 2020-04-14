%%***************************************************************************************
%% file         sfcn_can_receive_packed_mcb.m
%% brief        Block mask initialization function.
%%
%%---------------------------------------------------------------------------------------
%%                          C O P Y R I G H T
%%---------------------------------------------------------------------------------------
%%  Copyright 2013 (c) by HAN Automotive     http://www.han.nl     All rights reserved
%%
%%---------------------------------------------------------------------------------------
%%                            L I C E N S E
%%---------------------------------------------------------------------------------------
%% This file is part of the HANcoder Target Matlab/Simulink Blockset environment. For the
%% licensing terms, please contact HAN Automotive.
%%
%% This software has been carefully tested, but is not guaranteed for any particular
%% purpose. HAN Automotive does not offer any warranties and does not guarantee the 
%% accuracy, adequacy, or completeness of the software and is not responsible for any 
%% errors or omissions or the results obtained from use of the software.
%%
%%***************************************************************************************
function [] = sfcn_can_receive_packed_mcb(canBus, canID)

% Create resource keywords to be reserved in resource database
modelRTWFields = struct('canBus', int2str(canBus-1), 'canID', int2str(canID));

% Insert modelRTWFields in the I/O block S-Function containing the Tag starting with 'HANcoder_TARGET_'
HANcoder_TARGET_DataBlock = find_system(gcb, 'RegExp', 'on', 'FollowLinks', 'on', 'LookUnderMasks', 'all', 'BlockType', 'S-Function');
set_param(HANcoder_TARGET_DataBlock{1}, 'RTWdata', modelRTWFields);

%%******************************* end of sfcn_can_receive_packed_mcb.m ******************


