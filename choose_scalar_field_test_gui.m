function varargout = choose_scalar_field_test_gui(varargin)
% CHOOSE_SCALAR_FIELD_TEST_GUI MATLAB code for choose_scalar_field_test_gui.fig
%      CHOOSE_SCALAR_FIELD_TEST_GUI, by itself, creates a new CHOOSE_SCALAR_FIELD_TEST_GUI or raises the existing
%      singleton*.
%
%      H = CHOOSE_SCALAR_FIELD_TEST_GUI returns the handle to a new CHOOSE_SCALAR_FIELD_TEST_GUI or the handle to
%      the existing singleton*.
%
%      CHOOSE_SCALAR_FIELD_TEST_GUI('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in CHOOSE_SCALAR_FIELD_TEST_GUI.M with the given input arguments.
%
%      CHOOSE_SCALAR_FIELD_TEST_GUI('Property','Value',...) creates a new CHOOSE_SCALAR_FIELD_TEST_GUI or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before choose_scalar_field_test_gui_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to choose_scalar_field_test_gui_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help choose_scalar_field_test_gui

% Last Modified by GUIDE v2.5 19-Nov-2018 11:49:21

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
    'gui_Singleton',  gui_Singleton, ...
    'gui_OpeningFcn', @choose_scalar_field_test_gui_OpeningFcn, ...
    'gui_OutputFcn',  @choose_scalar_field_test_gui_OutputFcn, ...
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


% --- Executes just before choose_scalar_field_test_gui is made visible.
function choose_scalar_field_test_gui_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to choose_scalar_field_test_gui (see VARARGIN)

% Choose default command line output for choose_scalar_field_test_gui
handles.output = hObject;
assignin('base', 'base_handles_CSF_test' , handles)
% Update handles structure
guidata(hObject, handles);

% UIWAIT makes choose_scalar_field_test_gui wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = choose_scalar_field_test_gui_OutputFcn(hObject, eventdata, handles)
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on button press in pushbutton1.
function pushbutton1_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

if handles.radiobutton1.Value
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
elseif handles.radiobutton2.Value
    % set the scalarfieldselection to the corresponding desired value:
    ScalarFieldSelection = 2;
    % set field width to appropriate value for the desired field:
    FIELD_WIDTH= 300;
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
elseif handles.radiobutton3.Value
    figure()
    FIELD_WIDTH= 300;
    ax=gca;
    ax.XLim=[-FIELD_WIDTH FIELD_WIDTH];
    ax.YLim=[-FIELD_WIDTH FIELD_WIDTH];
    %cmap = hsv(N);
    res=100;
    xdivs=linspace(ax.XLim(1),ax.XLim(2),res);
    ydivs=linspace(ax.YLim(1),ax.YLim(2),res);
    ScalarFieldSelection = 3;
    [X,Y] = meshgrid(xdivs,ydivs);
    Z=readScalarFieldMulti(X,Y,ScalarFieldSelection);
    surf(X,Y,Z);
    title('Single Sink: Field Width= 300, Suggested Robot Velocity= 90')
    view([0 90])
else
    disp('No Value Selected')
end
