function varargout = BowlingGUI(varargin)
% BOWLINGGUI MATLAB code for BowlingGUI.fig
%      BOWLINGGUI, by itself, creates a new BOWLINGGUI or raises the existing
%      singleton*.
%
%      H = BOWLINGGUI returns the handle to a new BOWLINGGUI or the handle to
%      the existing singleton*.
%
%      BOWLINGGUI('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in BOWLINGGUI.M with the given input arguments.
%
%      BOWLINGGUI('Property','Value',...) creates a new BOWLINGGUI or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before BowlingGUI_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to BowlingGUI_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help BowlingGUI

% Last Modified by GUIDE v2.5 04-Jun-2017 18:37:34

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @BowlingGUI_OpeningFcn, ...
                   'gui_OutputFcn',  @BowlingGUI_OutputFcn, ...
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


% --- Executes just before BowlingGUI is made visible.
function BowlingGUI_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to BowlingGUI (see VARARGIN)

% Choose default command line output for BowlingGUI
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes BowlingGUI wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = BowlingGUI_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;



% --- Executes on slider movement.
function SliderVitesse_Callback(hObject, eventdata, handles)
% hObject    handle to SliderVitesse (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
SliderVitVal = get(hObject,'Value');
assignin('base','SliderVitVal',SliderVitVal);
set (handles.TextNumVit,'String',num2str(SliderVitVal));
% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider


% --- Executes during object creation, after setting all properties.
function SliderVitesse_CreateFcn(hObject, eventdata, handles)
% hObject    handle to SliderVitesse (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on slider movement.
function SliderPosition_Callback(hObject, eventdata, handles)
% hObject    handle to SliderPosition (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
SliderPosVal = get(hObject,'Value');
assignin('base','SliderPosVal',SliderPosVal);
set (handles.TextNumPos,'String',num2str(SliderPosVal));
% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider


% --- Executes during object creation, after setting all properties.
function SliderPosition_CreateFcn(hObject, eventdata, handles)
% hObject    handle to SliderPosition (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on slider movement.
function SliderAngle_Callback(hObject, eventdata, handles)
% hObject    handle to SliderAngle (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
SliderAngVal = get(hObject,'Value');
assignin('base','SliderAngVal',SliderAngVal);
set (handles.TextNumAng,'String',num2str(SliderAngVal));
% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider


% --- Executes during object creation, after setting all properties.
function SliderAngle_CreateFcn(hObject, eventdata, handles)
% hObject    handle to SliderAngle (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% -


% --- Executes on button press in VisualisationPush.
function VisualisationPush_Callback(hObject, eventdata, handles)
% hObject    handle to VisualisationPush (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

x = 17:0.1:19; % Décalage en hauteur pour avoir une vision peu déformée du triangle
a= get(handles.SliderPosition,'Value');
Rb=.11;
b= get(handles.SliderAngle,'Value');
y = a + x*sin((deg2rad(b))); % Trajectoire du centre de la boule
y1 = y + Rb; % Trajectoire d'une extrémité de la boule
y2 = y - Rb; % Trajectoire de l'autre extrémité
u = 0.5+1e-7*x; % Courbe de délimitation de la piste ( ne prend pas en compte les constantes donc j'ai pris une droite affine de pente quasi nulle.
d = -0.5+1e-7*x; % Idem ligne 165
axes (handles.Plot);
plot(y,x,'-r',y1,x,'-b',y2,x,'-b',u,x,'-black',d,x,'-black') % Tracé des trajectoires et de la piste
line([0,0.45], [18,18.79], 'Color', 'black');
line([0.45,-0.45], [18.79, 18.79], 'Color', 'black');
line([-0.45,0], [18.79, 18], 'Color', 'black'); % 169 à 171 tracé de l'espace occupé par les quilles

% --- Executes on button press in SimuPush.
function SimuPush_Callback(hObject, eventdata, handles)
% hObject    handle to SimuPush (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
a= get(handles.SliderPosition,'Value');
b= get(handles.SliderAngle,'Value');
c= get(handles.SliderVitesse,'Value');

N = 1e4;

Bowling_Interface(N, [c,deg2rad(b),-a]);
Affichage_Interface(N, 120);