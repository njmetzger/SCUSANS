function varargout = SCUSANS_GUI(varargin)
% SCUSANS_GUI MATLAB code for SCUSANS_GUI.fig
%      SCUSANS_GUI, by itself, creates a new SCUSANS_GUI or raises the existing
%      singleton*.
%
%      H = SCUSANS_GUI returns the handle to a new SCUSANS_GUI or the handle to
%      the existing singleton*.
%
%      SCUSANS_GUI('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in SCUSANS_GUI.M with the given input arguments.
%
%      SCUSANS_GUI('Property','Value',...) creates a new SCUSANS_GUI or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before SCUSANS_GUI_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to SCUSANS_GUI_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help SCUSANS_GUI

% Last Modified by GUIDE v2.5 31-Oct-2018 10:57:39

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @SCUSANS_GUI_OpeningFcn, ...
                   'gui_OutputFcn',  @SCUSANS_GUI_OutputFcn, ...
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

% --- Executes just before SCUSANS_GUI is made visible.
function SCUSANS_GUI_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to SCUSANS_GUI (see VARARGIN)

% Choose default command line output for SCUSANS_GUI
handles.output = hObject;

assignin('base', 'base_handles' , handles)

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes SCUSANS_GUI wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = SCUSANS_GUI_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% BEHAVIOR PUSHBUTTON CALLBACK FCNS %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% --- Executes on button press in cbox_Attract.
function cbox_Attract_Callback(hObject, eventdata, handles)
% hObject    handle to cbox_Attract (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% --- Executes on button press in cbox_Disperse.
function cbox_Disperse_Callback(hObject, eventdata, handles)
% hObject    handle to cbox_Disperse (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% --- Executes on button press in cbox_FindMin.
function cbox_FindMin_Callback(hObject, eventdata, handles)
% hObject    handle to cbox_FindMin (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% --- Executes on button press in cbox_FindMax.
function cbox_FindMax_Callback(hObject, eventdata, handles)
% hObject    handle to cbox_FindMax (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% --- Executes on button press in cbox_ContourFollow.
function cbox_ContourFollow_Callback(hObject, eventdata, handles)
% hObject    handle to cbox_ContourFollow (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% --- Executes on button press in cbox_RidgeFollow.
function cbox_RidgeFollow_Callback(hObject, eventdata, handles)
% hObject    handle to cbox_RidgeFollow (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% --- Executes on button press in SetBehaviors_PB.
function SetBehaviors_PB_Callback(hObject, eventdata, handles)
    % On Set Behaviors push button press, behaviors in
    % Swarm_Robot_Base_2018a are turned on/off based on paired checkboxes 
    
% Attract= num2str(handles.cbox_Attract.Value)

set_param('Swarm_Robot_Base_2018a/Robot 1 Behavior/Attract_Switch','sw',num2str(handles.cbox_Attract.Value))
set_param('Swarm_Robot_Base_2018a/Robot 1 Behavior/Disperse_Switch','sw',num2str(handles.cbox_Disperse.Value))
set_param('Swarm_Robot_Base_2018a/Robot 1 Behavior/FindMin_Switch','sw',num2str(handles.cbox_FindMin.Value))
set_param('Swarm_Robot_Base_2018a/Robot 1 Behavior/FindMax_Switch','sw',num2str(handles.cbox_FindMax.Value))
set_param('Swarm_Robot_Base_2018a/Robot 1 Behavior/FollowContour_Switch','sw',num2str(handles.cbox_ContourFollow.Value))
set_param('Swarm_Robot_Base_2018a/Robot 1 Behavior/FollowRidge_Switch','sw',num2str(handles.cbox_RidgeFollow.Value))



function numRobots_edit_Callback(hObject, eventdata, handles)
% hObject    handle to numRobots_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of numRobots_edit as text
%        str2double(get(hObject,'String')) returns contents of numRobots_edit as a double


% --- Executes during object creation, after setting all properties.
function numRobots_edit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to numRobots_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in setSimParam_PB.
function setSimParam_PB_Callback(hObject, eventdata, handles)
% hObject    handle to setSimParam_PB (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)



function SimRunTime_edit_Callback(hObject, eventdata, handles)
% hObject    handle to SimRunTime_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of SimRunTime_edit as text
%        str2double(get(hObject,'String')) returns contents of SimRunTime_edit as a double


% --- Executes during object creation, after setting all properties.
function SimRunTime_edit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to SimRunTime_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function SensorRange_edit_Callback(hObject, eventdata, handles)
% hObject    handle to SensorRange_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of SensorRange_edit as text
%        str2double(get(hObject,'String')) returns contents of SensorRange_edit as a double


% --- Executes during object creation, after setting all properties.
function SensorRange_edit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to SensorRange_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function AvoidanceRange_edit_Callback(hObject, eventdata, handles)
% hObject    handle to AvoidanceRange_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of AvoidanceRange_edit as text
%        str2double(get(hObject,'String')) returns contents of AvoidanceRange_edit as a double


% --- Executes during object creation, after setting all properties.
function AvoidanceRange_edit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to AvoidanceRange_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function DesiredContour_edit_Callback(hObject, eventdata, handles)
% hObject    handle to DesiredContour_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of DesiredContour_edit as text
%        str2double(get(hObject,'String')) returns contents of DesiredContour_edit as a double


% --- Executes during object creation, after setting all properties.
function DesiredContour_edit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to DesiredContour_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in runSim_PB.
function runSim_PB_Callback(hObject, eventdata, handles)
% hObject    handle to runSim_PB (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% set parameter values for switches based off behavior checkboxes 
set_param('Swarm_Robot_Base_2018a/Robot 1 Behavior/Attract_Switch','sw',num2str(handles.cbox_Attract.Value))
set_param('Swarm_Robot_Base_2018a/Robot 1 Behavior/Disperse_Switch','sw',num2str(handles.cbox_Disperse.Value))
set_param('Swarm_Robot_Base_2018a/Robot 1 Behavior/FindMin_Switch','sw',num2str(handles.cbox_FindMin.Value))
set_param('Swarm_Robot_Base_2018a/Robot 1 Behavior/FindMax_Switch','sw',num2str(handles.cbox_FindMax.Value))
set_param('Swarm_Robot_Base_2018a/Robot 1 Behavior/FollowContour_Switch','sw',num2str(handles.cbox_ContourFollow.Value))
set_param('Swarm_Robot_Base_2018a/Robot 1 Behavior/FollowRidge_Switch','sw',num2str(handles.cbox_RidgeFollow.Value))

% set simulation parameters based off text edit boxes: 
NUM_ROBOTS= str2double(handles.numRobots_edit.String);
SIM_TIME= str2double(handles.SimRunTime_edit.String);
SENSOR_RANGE= str2double(handles.SensorRange_edit.String);
AVOID_RANGE= str2double(handles.AvoidanceRange_edit.String);
DESIRED_VALUE= str2double(handles.DesiredContour_edit.String);

% run simulation: 
Swarm_Robot_Test_Sim(NUM_ROBOTS,SIM_TIME,SENSOR_RANGE,AVOID_RANGE,DESIRED_VALUE) 

