function varargout = Interface(varargin)



% INTERFACE MATLAB code for Interface.fig
%      INTERFACE, by itself, creates a new INTERFACE or raises the existing
%      singleton*.
%
%      H = INTERFACE returns the handle to a new INTERFACE or the handle to
%      the existing singleton*.
%
%      INTERFACE('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in INTERFACE.M with the given input arguments.
%
%      INTERFACE('Property','Value',...) creates a new INTERFACE or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before Interface_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to Interface_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help Interface

% Last Modified by GUIDE v2.5 05-Jun-2017 20:39:58

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @Interface_OpeningFcn, ...
                   'gui_OutputFcn',  @Interface_OutputFcn, ...
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


% --- Executes just before Interface is made visible.
function Interface_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to Interface (see VARARGIN)

% Choose default command line output for Interface
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes Interface wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = Interface_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on slider movement.
function slider_speed_Callback(hObject, eventdata, handles)
% hObject    handle to slider_speed (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
value = get(hObject,'Value');
str_value = num2str(value);
disp = strcat(str_value,' m/s');
set(handles.text_speed,'String', disp);


% --- Executes during object creation, after setting all properties.
function slider_speed_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider_speed (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on slider movement.
function slider_angle_Callback(hObject, eventdata, handles)
% hObject    handle to slider_angle (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
value = get(hObject,'Value');
str_value = num2str(value);
disp = strcat(str_value,' rad');
set(handles.text_angle,'String', disp);



% --- Executes during object creation, after setting all properties.
function slider_angle_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider_angle (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on slider movement.
function slider_offset_Callback(hObject, eventdata, handles)
% hObject    handle to slider_offset (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
value = get(hObject,'Value');
str_value = num2str(value);
disp = strcat(str_value,' m');
set(handles.text_offset,'String', disp);


% --- Executes during object creation, after setting all properties.
function slider_offset_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider_offset (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on button press in button_process.
function button_process_Callback(hObject, eventdata, handles)

global R_b R_bas_q R_haut_q R_q Nb_q

% hObject    handle to button_process (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
speed = get(handles.slider_speed, 'Value');
angle = get(handles.slider_angle, 'Value');
offset = get(handles.slider_offset, 'Value');

Nb_q = 10;

N = 1e4;
time = [0, 4];
Nb_images_per_sec = 120;

R_q=0.055;                  %rayon de la quille (m) dans le cadre d'une approximation par un cylindre
R_bas_q=R_q;                %rayon de la boule inférieure (m)
R_haut_q=3*R_q/4;           %rayon de la boule supérieure (m)

R_b=0.15;                   %rayon boule (m)

Bowling_Interface(N, [speed, angle, offset], time);

Affichage_Interface(handles.axes_bowling, N, Nb_images_per_sec, time);
