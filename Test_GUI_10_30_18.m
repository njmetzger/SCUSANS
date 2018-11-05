function varargout = Test_GUI_10_30_18(varargin)
% TEST_GUI_10_30_18 MATLAB code for Test_GUI_10_30_18.fig
%      TEST_GUI_10_30_18, by itself, creates a new TEST_GUI_10_30_18 or raises the existing
%      singleton*.
%
%      H = TEST_GUI_10_30_18 returns the handle to a new TEST_GUI_10_30_18 or the handle to
%      the existing singleton*.
%
%      TEST_GUI_10_30_18('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in TEST_GUI_10_30_18.M with the given input arguments.
%
%      TEST_GUI_10_30_18('Property','Value',...) creates a new TEST_GUI_10_30_18 or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before Test_GUI_10_30_18_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to Test_GUI_10_30_18_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help Test_GUI_10_30_18

% Last Modified by GUIDE v2.5 30-Oct-2018 18:40:27

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
    'gui_Singleton',  gui_Singleton, ...
    'gui_OpeningFcn', @Test_GUI_10_30_18_OpeningFcn, ...
    'gui_OutputFcn',  @Test_GUI_10_30_18_OutputFcn, ...
    'gui_LayoutFcn',  [] , ...
    'gui_Callback',   []);
if nargin && ischar(varargin{1})
    gui_State.gui_Callback = str2func(varargin{1});
end

if nargout
    [varargout{1:nargout}] = gui_mainfcn(gui_State, varargin{:});
else
    gui_mainfcn(gui_State, varargin{:});
end
% End initialization code - DO NOT EDIT

% --- Executes just before Test_GUI_10_30_18 is made visible.
function Test_GUI_10_30_18_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to Test_GUI_10_30_18 (see VARARGIN)

% Choose default command line output for Test_GUI_10_30_18
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);


% UIWAIT makes Test_GUI_10_30_18 wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = Test_GUI_10_30_18_OutputFcn(hObject, eventdata, handles)
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% PUSH BUTTON CALLBACKS FOR BEHAVIOR SETTING
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% --- Executes on button press in cbox_Attract.
function cbox_Attract_Callback(hObject, eventdata, handles)
% hObject    handle to cbox_Attract (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% Attract=get(hObject,'Value');
% if Attract
%     disp('attract activated')
%     set_param('Swarm_Robot_Base_2018a/Robot 1 Behavior/Attract_Switch','sw','1')
% else
%     set_param('Swarm_Robot_Base_2018a/Robot 1 Behavior/Attract_Switch','sw','0')
% end

% Hint: get(hObject,'Value') returns toggle state of cbox_Attract


% --- Executes on button press in disperse.
function disperse_Callback(hObject, eventdata, handles)
% hObject    handle to disperse (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of disperse
% Disperse=get(hObject,'Value');
% if Disperse
%     disp('Disperse activated');
%     set_param('Swarm_Robot_Base_2018a/Robot 1 Behavior/Disperse_Switch','sw','1')
% else
%     disp('disperse de-activated');
%     set_param('Swarm_Robot_Base_2018a/Robot 1 Behavior/Disperse_Switch','sw','0')
% end

% --- Executes on button press in findMin.
function findMin_Callback(hObject, eventdata, handles)
% hObject    handle to findMin (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% FindMin=get(hObject,'Value');
% if FindMin
%     disp('Find Min activated')
%     set_param('Swarm_Robot_Base_2018a/Robot 1 Behavior/FindMin_Switch','sw','1')
% else
%     disp('Fine Min de-activated')
%     set_param('Swarm_Robot_Base_2018a/Robot 1 Behavior/FindMin_Switch','sw','0')
% end

%--- Executes on button press in findMax.
function findMax_Callback(hObject, eventdata, handles)
% hObject    handle to findMax (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% findMax=get(hObject,'Value');
% if findMax
%     disp('Find Max activated')
%     set_param('Swarm_Robot_Base_2018a/Robot 1 Behavior/FindMax_Switch','sw','1')
%     
% else
%     disp('Find Max de-activated')
%     set_param('Swarm_Robot_Base_2018a/Robot 1 Behavior/FindMax_Switch','sw','0')
% end

% --- Executes on button press in contourFollow.
% function contourFollow_Callback(hObject, eventdata, handles)
% hObject    handle to contourFollow (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% contourFollow=get(hObject,'Value');
% if contourFollow
%     disp('Contour Finding activated')
%     set_param('Swarm_Robot_Base_2018a/Robot 1 Behavior/FollowContour_Switch','sw','1')
% else
%     disp('Contour Finding de-activated')
%     set_param('Swarm_Robot_Base_2018a/Robot 1 Behavior/FollowContour_Switch','sw','0')
% end

function ridgeFollow_Callback(hObject, eventdata, handles)
% hObject    handle to ridgeFollow (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% ridgeFollow=get(hObject,'Value');
% if ridgeFollow
%     disp('Ridge Following activated')
%     set_param('Swarm_Robot_Base_2018a/Robot 1 Behavior/FollowRidge_Switch','sw','1')
% else
%     disp('Ridge Following de-activated')
%     set_param('Swarm_Robot_Base_2018a/Robot 1 Behavior/FollowRidge_Switch','sw','0')
% end


% --- Executes on button press in runSimPB.
function runSimPB_Callback(hObject, eventdata, handles)
% hObject    handle to runSimPB (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)



function edit1_Callback(hObject, eventdata, handles)
% hObject    handle to edit1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% --- Executes during object creation, after setting all properties.
function edit1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function SensorRange_Edit_Callback(hObject, eventdata, handles)
% hObject    handle to SensorRange_Edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
SensorRange=str2double(get(hObject,'String'));
disp('Sensor Range is ')
disp(SensorRange)



% --- Executes during object creation, after setting all properties.
function SensorRange_Edit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to SensorRange_Edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function Num_Robots_Edit_Callback(hObject, eventdata, handles)
% hObject    handle to Num_Robots_Edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
NUM_ROBOTS=str2double(get(hObject,'String'));
disp('Number of Robots is ')
disp(NUM_ROBOTS)

% --- Executes during object creation, after setting all properties.
function Num_Robots_Edit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Num_Robots_Edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function sim_time_edit_Callback(hObject, eventdata, handles)
% hObject    handle to sim_time_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
SIM_TIME=str2double(get(hObject,'String'));
disp('Simulation Run Time is ')
disp(SIM_TIME)
% Hints: get(hObject,'String') returns contents of sim_time_edit as text
%        str2double(get(hObject,'String')) returns contents of sim_time_edit as a double


% --- Executes during object creation, after setting all properties.
function sim_time_edit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to sim_time_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function AvoidRange_Edit_Callback(hObject, eventdata, handles)
% hObject    handle to AvoidRange_Edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
AvoidRange=str2double(get(hObject,'String'));
disp('Avoidance Range is ')
disp(AvoidRange)
% Hints: get(hObject,'String') returns contents of AvoidRange_Edit as text
%        str2double(get(hObject,'String')) returns contents of AvoidRange_Edit as a double


% --- Executes during object creation, after setting all properties.
function AvoidRange_Edit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to AvoidRange_Edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function DesiredContour_Edit_Callback(hObject, eventdata, handles)
% hObject    handle to DesiredContour_Edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
DesiredValue=str2double(get(hObject,'String'));
disp('Desired Contour Value is ')
disp(DesiredValue)
% Hints: get(hObject,'String') returns contents of DesiredContour_Edit as text
%        str2double(get(hObject,'String')) returns contents of DesiredContour_Edit as a double


% --- Executes during object creation, after setting all properties.
function DesiredContour_Edit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to DesiredContour_Edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- If Enable == 'on', executes on mouse press in 5 pixel border.
% --- Otherwise, executes on mouse press in 5 pixel border or over simModeOn_RB.
function simModeOn_RB_ButtonDownFcn(hObject, eventdata, handles)
% hObject    handle to simModeOn_RB (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
simModeOn= get(hObject, 'Value')
if simModeOn
    disp('Simulation Mode Active, Testbed De-Activated')
else
    disp('Testbed Mode Active, Simulation De-Activated')
end


% --- Executes on button press in simModeOn_RB.
function simModeOn_RB_Callback(hObject, eventdata, handles)
% hObject    handle to simModeOn_RB (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
simModeOn= get(hObject, 'Value');
if simModeOn
    disp('Simulation Mode Active, Testbed De-Activated')
end


% Hint: get(hObject,'Value') returns toggle state of simModeOn_RB


% --- Executes on button press in testBedOn_RB.
function testBedOn_RB_Callback(hObject, eventdata, handles)
% hObject    handle to testBedOn_RB (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
testBedOn= get(hObject, 'Value');
if testBedOn
    disp('Testbed Mode Active, Simulation De-Activated')
end


% Hint: get(hObject,'Value') returns toggle state of testBedOn_RB


% --- Executes on button press in pushbutton2.
function pushbutton2_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
test_value= get(hObject, 'Value');
if test_value
    disp('Button pressed')
end


% --- Executes on button press in checkbox8.
function checkbox8_Callback(hObject, eventdata, handles)
% hObject    handle to checkbox8 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of checkbox8


% --- Executes on button press in checkbox9.
function checkbox9_Callback(hObject, eventdata, handles)
% hObject    handle to checkbox9 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of checkbox9


% --- Executes on button press in checkbox10.
function checkbox10_Callback(hObject, eventdata, handles)
% hObject    handle to checkbox10 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of checkbox10


% --- Executes on button press in checkbox11.
function checkbox11_Callback(hObject, eventdata, handles)
% hObject    handle to checkbox11 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of checkbox11


% --- Executes on button press in sunglowCheckbox.
function sunglowCheckbox_Callback(hObject, eventdata, handles)
% hObject    handle to sunglowCheckbox (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of sunglowCheckbox


% --- Executes on button press in redwoodCheckbox.
function redwoodCheckbox_Callback(hObject, eventdata, handles)
% hObject    handle to redwoodCheckbox (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of redwoodCheckbox


% --- Executes on button press in canaryCheckbox.
function canaryCheckbox_Callback(hObject, eventdata, handles)
% hObject    handle to canaryCheckbox (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of canaryCheckbox


% --- Executes on button press in pacBlueCheckbox.
function pacBlueCheckbox_Callback(hObject, eventdata, handles)
% hObject    handle to pacBlueCheckbox (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of pacBlueCheckbox


% --- Executes on button press in setRobotsPB.
function setRobotsPB_Callback(hObject, eventdata, handles)
% hObject    handle to setRobotsPB (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in setBehaviorsPB.
function setBehaviorsPB_Callback(hObject, eventdata, handles)
% hObject    handle to setBehaviorsPB (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% disp('Set Behaviors Button Pressed')
% on button press, get values for all the different 
AttractValue= get(handles.cbox_Attract.Value);
disp(AttractValue)
cbox_data_type= class('AttractValue')
disp('cbox_data_type') 


%set_param('Swarm_Robot_Base_2018a/Robot 1 Behavior/Attract_Switch','sw',(get(handles.cbox_Attract.Value)))
% set_param('Swarm_Robot_Base_2018a/Robot 1 Behavior/Disperse_Switch','sw', num2str(get(handles.disperse.Value)))
% set_param('Swarm_Robot_Base_2018a/Robot 1 Behavior/FindMin_Switch','sw', num2str(get(handles.findMin.Value)))
% set_param('Swarm_Robot_Base_2018a/Robot 1 Behavior/FindMax_Switch','sw', num2str(get(handles.findMax.Value)))
% set_param('Swarm_Robot_Base_2018a/Robot 1 Behavior/FollowContour_Switch','sw', num2str(get(handles.contourFollow.Value)))
% set_param('Swarm_Robot_Base_2018a/Robot 1 Behavior/FollowRidge_Switch','sw', num2str(get(handles.ridgeFollow.Value)))
