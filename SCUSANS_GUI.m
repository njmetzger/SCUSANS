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

% Last Modified by GUIDE v2.5 22-Feb-2019 15:49:40

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
% Update handles structure
guidata(hObject, handles);
set(handles.ExpRobotSelect,'Visible','off')

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


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% ROBOT NAME CHECKLIST CALLBACK FCNS %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% --- Executes on button press in cbox_Pink.
function cbox_Pink_Callback(hObject, eventdata, handles)
% hObject    handle to cbox_Pink (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% --- Executes on button press in cbox_Celeste.
function cbox_Celeste_Callback(hObject, eventdata, handles)
% hObject    handle to cbox_Celeste (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% --- Executes on button press in cbox_Pacific_Blue.
function cbox_Pacific_Blue_Callback(hObject, eventdata, handles)
% hObject    handle to cbox_Pacific_Blue (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% --- Executes on button press in cbox_Canary.
function cbox_Canary_Callback(hObject, eventdata, handles)
% hObject    handle to cbox_Canary (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% --- Executes on button press in cbox_Sunglow.
function cbox_Sunglow_Callback(hObject, eventdata, handles)
% hObject    handle to cbox_Sunglow (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% --- Executes on button press in cbox_Schwein.
function cbox_Schwein_Callback(hObject, eventdata, handles)
% hObject    handle to cbox_Schwein (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% --- Executes on button press in cbox_Redwood.
function cbox_Redwood_Callback(hObject, eventdata, handles)
% hObject    handle to cbox_Redwood (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% --- Executes on button press in cbox_Watermelon.
function cbox_Watermelon_Callback(hObject, eventdata, handles)
% hObject    handle to cbox_Watermelon (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% --- Executes on button press in cbox_Tidal.
function cbox_Tidal_Callback(hObject, eventdata, handles)
% hObject    handle to cbox_Tidal (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% --- Executes on button press in cbox_Wisteria.
function cbox_Wisteria_Callback(hObject, eventdata, handles)
% hObject    handle to cbox_Wisteria (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% --- Executes on button press in SetBehaviors_PB.

function SetBehaviors_PB_Callback(hObject, eventdata, handles)
    % On Set Behaviors push button press, behaviors in
    % Swarm_Robot_Base_a are turned on/off based on paired checkboxes
base = handleVersion(handles);

% Attract= num2str(handles.cbox_Attract.Value)
set_param(strcat(base,'/Robot 1 Behavior/Attract_Switch'),'sw',num2str(handles.cbox_Attract.Value))
set_param(strcat(base,'/Robot 1 Behavior/Disperse_Switch'),'sw',num2str(handles.cbox_Disperse.Value))
set_param(strcat(base,'/Robot 1 Behavior/FindMin_Switch'),'sw',num2str(handles.cbox_FindMin.Value))
set_param(strcat(base,'/Robot 1 Behavior/FindMax_Switch'),'sw',num2str(handles.cbox_FindMax.Value))
set_param(strcat(base,'/Robot 1 Behavior/FollowContour_Switch'),'sw',num2str(handles.cbox_ContourFollow.Value))



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
base = handleVersion(handles);

% set parameter values for switches based off behavior checkboxes 
set_param(strcat(base,'/Robot 1 Behavior/Attract_Switch'),'sw',num2str(handles.cbox_Attract.Value))
set_param(strcat(base,'/Robot 1 Behavior/Disperse_Switch'),'sw',num2str(handles.cbox_Disperse.Value))
set_param(strcat(base,'/Robot 1 Behavior/FindMin_Switch'),'sw',num2str(handles.cbox_FindMin.Value))
set_param(strcat(base,'/Robot 1 Behavior/FindMax_Switch'),'sw',num2str(handles.cbox_FindMax.Value))
set_param(strcat(base,'/Robot 1 Behavior/FollowContour_Switch'),'sw',num2str(handles.cbox_ContourFollow.Value))
set_param(strcat(base,'/Robot 1 Behavior/GoTo_Switch'),'sw',num2str(handles.cbox_GoTo.Value))

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
SIM_TIME= str2double(handles.SimRunTime_edit.String);
SENSOR_RANGE= str2double(handles.SensorRange_edit.String);
AVOID_RANGE= str2double(handles.AvoidanceRange_edit.String);
DESIRED_VALUE= str2double(handles.DesiredContour_edit.String);
CONTOUR_BUFFER= str2double(handles.contourBuffer_edit.String);
% ROBOT_SPEED= str2double(handles.robSpeed_edit.String); 

x_init_center= str2double(handles.initCond_centerX_edit.String); 
y_init_center= str2double(handles.initCond_centerY_edit.String); 
init_radius= str2double(handles.initCond_radius_edit.String); 
GoTo_Coords = [str2double(handles.goTo_X_Coord_edit.String), str2double(handles.goTo_X_Coord_edit.String)]; 

%because robot_speed is not a parameter used inside swarm_robot_test_sim,
%update robot speed here:

set_param(strcat(base,'/Robot 1 Behavior/Robot Speed'),'value', handles.robSpeed_edit.String);
set_param(strcat(base,'/Robot 1 Behavior/GoTo_X'),'value',handles.goTo_X_Coord_edit.String);
set_param(strcat(base,'/Robot 1 Behavior/GoTo_Y'),'value',handles.goTo_Y_Coord_edit.String);

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
elseif handles.tbSink_RB.Value
    ScalarFieldSelection = 4;
else
    disp('No Value Selected')
end 

if handles.SelectSimRB.Value
    disp('sim')
    robots = []; 
    isExp = false;
    NUM_ROBOTS= str2double(handles.numRobots_edit.String);
elseif handles.SelectTestbedRB.Value 
    disp('exp')
    isExp = true; 
    all_robots = [string('canary'), string('celeste'),...
        string('pacific-blue'), string('pink'), string('redwood'), ...
    string('schweinefleisch'), string('sunglow'), string('tidal'), ...
    string('watermelon'),string('wisteria'),];
    robot_select = [handles.cbox_Canary.Value,handles.cbox_Celeste.Value,...
        handles.cbox_Pacific_Blue.Value,handles.cbox_Pink.Value, ...
        handles.cbox_Redwood.Value, handles.cbox_Schwein.Value,...
        handles.cbox_Sunglow.Value,handles.cbox_Tidal.Value,...
        handles.cbox_Watermelon.Value,handles.cbox_Wisteria.Value];
    robots = all_robots(logical(robot_select));
    NUM_ROBOTS = length(robots);
end
% run simulation: 
Swarm_Robot_Test_Sim(NUM_ROBOTS,SIM_TIME,SENSOR_RANGE,AVOID_RANGE,DESIRED_VALUE,CONTOUR_BUFFER,ScalarFieldSelection,behavior,x_init_center,y_init_center,init_radius,isExp, robots,base) 

function contourBuffer_edit_Callback(hObject, eventdata, handles)
% hObject    handle to contourBuffer_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

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
    p_title ='Composite Scalar Field: Field Width= 300, Suggested Robot Speed= 90';
elseif handles.singSource_RB.Value
    % set the scalarfieldselection to the corresponding desired value:
    ScalarFieldSelection = 2;
    % set field width to appropriate value for the desired field:
    FIELD_WIDTH= 300;   
    p_title ='Single Source: Field Width= 300, Suggested Robot Velocity= 90';
elseif handles.singSink_RB.Value
    % Set the scalarfieldselection to the corresponding desired value: 
    ScalarFieldSelection = 3;
    % set the field width to appropriate value for the desired field: 
    FIELD_WIDTH= 300;  
    p_title = 'Single Sink: Field Width= 300, Suggested Robot Velocity= 90';
elseif handles.tbSink_RB.Value
    % Set the scalarfieldselection to the corresponding desired value: 
    ScalarFieldSelection = 4;
    % set the field width to appropriate value for the desired field: 
    FIELD_WIDTH= 5;  
    p_title = 'Testbed Single Sink: Field Width= 5, Suggested Robot Velocity= 0.5';
else
    disp('No Value Selected')
end
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
title(p_title)
view([0 90])

function robSpeed_edit_Callback(hObject, eventdata, handles)
% hObject    handle to robSpeed_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

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

% --- Executes on button press in SelectSimRB.
function SelectSimRB_Callback(hObject, eventdata, handles)
% hObject    handle to SelectSimRB (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of SelectSimRB
if handles.SelectSimRB.Value
    set(handles.ExpRobotSelect,'Visible','off')
    set(handles.numRobots_edit,'Enable','on')
    set(handles.MLVersion_box,'Visible','on')
end

% --- Executes on button press in SelectTestbedRB.
function SelectTestbedRB_Callback(hObject, eventdata, handles)
% hObject    handle to SelectTestbedRB (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of SelectTestbedRB
if handles.SelectTestbedRB.Value
    set(handles.ExpRobotSelect,'Visible','on')
    set(handles.numRobots_edit,'Enable','off')
    set(handles.MLVersion_box,'Visible','off')
end

%helper function to manage version 
function [base] = handleVersion(handles)
    if handles.SelectTestbedRB.Value
        base = 'Swarm_Robot_Base_2016b';
        if exist('Swarm_Robot_Base_2018a')
            close_system('Swarm_Robot_Base_2018a',0)
        end
        if exist('Swarm_Robot_Base_2018b')
            close_system('Swarm_Robot_Base_2018b',0)
        end 
    elseif handles.ML2018a.Value
        base = 'Swarm_Robot_Base_2018a';
        if exist('Swarm_Robot_Base_2018b')
            close_system('Swarm_Robot_Base_2018b',0)
        end
        if exist('Swarm_Robot_Base_2016b')
            close_system('Swarm_Robot_Base_2016b',0)
        end 
    else
        base = 'Swarm_Robot_Base_2018b';
        if exist('Swarm_Robot_Base_2018a')
            close_system('Swarm_Robot_Base_2018a',0)
        end
        if exist('Swarm_Robot_Base_2016b')
            close_system('Swarm_Robot_Base_2016b',0)
        end 
    end
    open(strcat(base,'.slx'))


% --- Executes on button press in cbox_GoTo.
function cbox_GoTo_Callback(hObject, eventdata, handles)
% hObject    handle to cbox_GoTo (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of cbox_GoTo



function goTo_X_Coord_edit_Callback(hObject, eventdata, handles)
% hObject    handle to goTo_X_Coord_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of goTo_X_Coord_edit as text
%        str2double(get(hObject,'String')) returns contents of goTo_X_Coord_edit as a double


% --- Executes during object creation, after setting all properties.
function goTo_X_Coord_edit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to goTo_X_Coord_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function goTo_Y_Coord_edit_Callback(hObject, eventdata, handles)
% hObject    handle to goTo_Y_Coord_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of goTo_Y_Coord_edit as text
%        str2double(get(hObject,'String')) returns contents of goTo_Y_Coord_edit as a double


% --- Executes during object creation, after setting all properties.
function goTo_Y_Coord_edit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to goTo_Y_Coord_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in clearGraphs_PB.
function clearGraphs_PB_Callback(hObject, eventdata, handles)
% hObject    handle to clearGraphs_PB (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
all_figs = findobj(0, 'type', 'figure'); 
for i=1:length(all_figs)
    if ~strcmp(all_figs(i).Name, 'SCUSANS_GUI')
        close(all_figs(i))
    end
end
