%%***************************************************************************************
%% file         sfcn_controller_error_readerror.m
%% brief        Level-2 M file S-Function for reading an error.
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
function sfcn_controller_error_readerror(block)
  setup(block);
%endfunction


%% Function: setup ===================================================
%% Abstract:
%%   Set up the S-function block's basic characteristics such as:
%%   - Input ports
%%   - Output ports
%%   - Dialog parameters
%%   - Options
%% 
%%   Required         : Yes
%%   C-Mex counterpart: mdlInitializeSizes
function setup(block)
  %% Register number of input and output ports
  block.NumInputPorts = 1;
  block.NumOutputPorts = 5;
  %% Override ports
  block.InputPort(1).Dimensions = 1;
  block.InputPort(1).DatatypeID = 3; %% uint8_T is type 3, see rtwtypes.h
  block.InputPort(1).Complexity = 'Real';
  block.InputPort(1).DirectFeedthrough = true;  %% We will not use the direct (.Data) value of the input to calculate the direct (.Data) value of the output
  block.InputPort(1).SamplingMode = 'sample';
  for outputCounter = 1:5
     block.OutputPort(outputCounter).Dimensions = 1;
     block.OutputPort(outputCounter).Complexity = 'Real';
     block.OutputPort(outputCounter).SamplingMode = 'sample';
  end
  block.OutputPort(1).DatatypeID = 3; %% uint8_T is type 3, see rtwtypes.h
  block.OutputPort(2).DatatypeID = 5; %% uint16_T is type 5, see rtwtypes.h
  block.OutputPort(3).DatatypeID = 3; %% uint8_T is type 3, see rtwtypes.h
  block.OutputPort(4).DatatypeID = 3; %% uint8_T is type 3, see rtwtypes.h
  block.OutputPort(5).DatatypeID = 7; %% uint32_T is type 7, see rtwtypes.h
  % Number of S-Function parameters expected
  % tsamp
  block.NumDialogPrms = 1;
  block.SampleTimes = [block.DialogPrm(1).Data 0];
  %% -----------------------------------------------------------------
  %% Register methods called at run-time
  %% -----------------------------------------------------------------
  
  %% 
  %% Start:
  %%   Functionality    : Called in order to initialize state and work
  %%                      area values
  %%   C-Mex counterpart: mdlStart
  %%
  block.RegBlockMethod('Start', @Start);

  %% 
  %% Outputs:
  %%   Functionality    : Called to generate block outputs in
  %%                      simulation step
  %%   C-Mex counterpart: mdlOutputs
  %%
  block.RegBlockMethod('Outputs', @Outputs);

  %% 
  %% Update:
  %%   Functionality    : Called to update discrete states
  %%                      during simulation step
  %%   C-Mex counterpart: mdlUpdate
  %%
  block.RegBlockMethod('Update', @Update);
%endfunction

function Start(block)

  %% No start

%endfunction


function Outputs(block)

  %% No output
  
%endfunction


function Update(block)

  %% No update
  
%endfunction


%%******************************* end of sfcn_controller_error_readerror.m **************
