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

% Last Modified by GUIDE v2.5 24-Nov-2018 11:20:42

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

%assignin('base', 'base_handles' , handles)
open('Swarm_Robot_Base_2018a.slx') 
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

% set behavior switch used to plot time histories of robots: 
if handles.cbox_FindMin.Value && handles.cbox_FindMax.Value 
    behavior = 'Incompatible'
elseif handles.cbox_FindMin.Value && handles.cbox_ContourFollow.Value 
    behavior = 'Incompatible'
elseif handles.cbox_FindMax.Value && handles.cbox_ContourFollow.Value  
    behavior = 'Incompatible'
elseif handles.cbox_FindMax.Value && handles.cbox_ContourFollow.Value && handles.cbox_FindMin.Value
    behavior = 'Incompatible'
elseif handles.cbox_FindMin.Value 
    behavior = 'Find Min' 
elseif handles.cbox_FindMax.Value 
    behavior = 'Find Max' 
elseif handles.cbox_ContourFollow.Value
    behavior = 'Contour Following' 
else 
    behavior = 'Null' 
end 

% set simulation parameters based off text edit boxes: 
NUM_ROBOTS= str2double(handles.numRobots_edit.String);
SIM_TIME= str2double(handles.SimRunTime_edit.String);
SENSOR_RANGE= str2double(handles.SensorRange_edit.String);
AVOID_RANGE= str2double(handles.AvoidanceRange_edit.String);
DESIRED_VALUE= str2double(handles.DesiredContour_edit.String);
CONTOUR_BUFFER= str2double(handles.contourBuffer_edit.String);
% ROBOT_SPEED= str2double(handles.robSpeed_edit.String); 

x_init_center= str2double(handles.initCond_centerX_edit.String) 
y_init_center= str2double(handles.initCond_centerY_edit.String) 
init_radius= str2double(handles.initCond_radius_edit.String) 

%because robot_speed is not a parameter used inside swarm_robot_test_sim,
%update robot speed here: 

set_param('Swarm_Robot_Base_2018a/Robot 1 Behavior/Robot Speed','value', handles.robSpeed_edit.String);

% determine Scalar Field to use based off the radio button group: 
% To add a scalar field, make sure that it is added in three places: 
%     1) readScalarField function 
%     2) SCUSANS_GUI as a radio button 
%     3) list below so it can be passed to the test_sim file
if handles.compField_RB.Value
    ScalarFieldSelection = 1;
elseif handles.singSource_RB.Value
    ScalarFieldSelection = 2;
elseif handles.singSink_RB.Value
    ScalarFieldSelection = 3;
else
    disp('No Value Selected')
end 

% run simulation: 
Swarm_Robot_Test_Sim(NUM_ROBOTS,SIM_TIME,SENSOR_RANGE,AVOID_RANGE,DESIRED_VALUE,CONTOUR_BUFFER,ScalarFieldSelection,behavior,x_init_center,y_init_center,init_radius) 



function contourBuffer_edit_Callback(hObject, eventdata, handles)
% hObject    handle to contourBuffer_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of contourBuffer_edit as text
%        str2double(get(hObject,'String')) returns contents of contourBuffer_edit as a double


% --- Executes during object creation, after setting all properties.
function contourBuffer_edit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to contourBuffer_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in prevScalarField_PB.
function prevScalarField_PB_Callback(hObject, eventdata, handles)
% hObject    handle to prevScalarField_PB (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
if handles.compField_RB.Value
    % set the scalarfieldselection to the corresponding desired value:
    ScalarFieldSelection = 1;
    % set field width to appropriate value for the desired field:
    FIELD_WIDTH= 300;
    % use the same plotting logic as is used in Swarm_Robot_Test_Sim
    figure()
    ax=gca;
    ax.XLim=[-FIELD_WIDTH FIELD_WIDTH];
    ax.YLim=[-FIELD_WIDTH FIELD_WIDTH];
    %cmap = hsv(N);
    res=100;
    xdivs=linspace(ax.XLim(1),ax.XLim(2),res);
    ydivs=linspace(ax.YLim(1),ax.YLim(2),res);
    [X,Y] = meshgrid(xdivs,ydivs);
    Z=readScalarField(X,Y,ScalarFieldSelection);
    surf(X,Y,Z);
    title('Composite Scalar Field: Field Width= 300, Suggested Robot Speed= 90')
    view([0 90])
elseif handles.singSource_RB.Value
    % set the scalarfieldselection to the corresponding desired value:
    ScalarFieldSelection = 2;
    % set field width to appropriate value for the desired field:
    FIELD_WIDTH= 300;   
    % use the same plotting logic as is used in Swarm_Robot_Test_Sim
    figure()
    ax=gca;
    ax.XLim=[-FIELD_WIDTH FIELD_WIDTH];
    ax.YLim=[-FIELD_WIDTH FIELD_WIDTH];
    %cmap = hsv(N);
    res=100;
    xdivs=linspace(ax.XLim(1),ax.XLim(2),res);
    ydivs=linspace(ax.YLim(1),ax.YLim(2),res);
    [X,Y] = meshgrid(xdivs,ydivs);
    Z=readScalarFieldMulti(X,Y,ScalarFieldSelection);
    surf(X,Y,Z);
    title('Single Source: Field Width= 300, Suggested Robot Velocity= 90')
    view([0 90])
elseif handles.singSink_RB.Value
    % Set the scalarfieldselection to the corresponding desired value: 
    ScalarFieldSelection = 3;
    % set the field width to appropriate value for the desired field: 
    FIELD_WIDTH= 300;  
    % use the same plotting logic as is used in Swarm_Robot_Test_Sim
    figure()
    ax=gca;
    ax.XLim=[-FIELD_WIDTH FIELD_WIDTH];
    ax.YLim=[-FIELD_WIDTH FIELD_WIDTH];
    %cmap = hsv(N);
    res=100;
    xdivs=linspace(ax.XLim(1),ax.XLim(2),res);
    ydivs=linspace(ax.YLim(1),ax.YLim(2),res);
    [X,Y] = meshgrid(xdivs,ydivs);
    Z=readScalarFieldMulti(X,Y,ScalarFieldSelection);
    surf(X,Y,Z);
    title('Single Sink: Field Width= 300, Suggested Robot Velocity= 90')
    view([0 90])
else
    disp('No Value Selected')
end



function robSpeed_edit_Callback(hObject, eventdata, handles)
% hObject    handle to robSpeed_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of robSpeed_edit as text
%        str2double(get(hObject,'String')) returns contents of robSpeed_edit as a double


% --- Executes during object creation, after setting all properties.
function robSpeed_edit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to robSpeed_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function initCond_radius_edit_Callback(hObject, eventdata, handles)
% hObject    handle to initCond_radius_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of initCond_radius_edit as text
%        str2double(get(hObject,'String')) returns contents of initCond_radius_edit as a double


% --- Executes during object creation, after setting all properties.
function initCond_radius_edit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to initCond_radius_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function initCond_centerX_edit_Callback(hObject, eventdata, handles)
% hObject    handle to initCond_centerX_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of initCond_centerX_edit as text
%        str2double(get(hObject,'String')) returns contents of initCond_centerX_edit as a double


% --- Executes during object creation, after setting all properties.
function initCond_centerX_edit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to initCond_centerX_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function initCond_centerY_edit_Callback(hObject, eventdata, handles)
% hObject    handle to initCond_centerY_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of initCond_centerY_edit as text
%        str2double(get(hObject,'String')) returns contents of initCond_centerY_edit as a double


% --- Executes during object creation, after setting all properties.
function initCond_centerY_edit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to initCond_centerY_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
