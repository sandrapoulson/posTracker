function varargout = posTracker(varargin)
% POSTRACKER M-file for posTracker.fig
%      POSTRACKER, by itself, creates a new POSTRACKER or raises the existing
%      singleton*.
%
%      H = POSTRACKER returns the handle to a new POSTRACKER or the handle to
%      the existing singleton*.
%
%      POSTRACKER('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in POSTRACKER.M with the given input arguments.
%
%      POSTRACKER('Property','Value',...) creates a new POSTRACKER or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before posTracker_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed t6o posTracker_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help posTracker

% Last Modified by GUIDE v2.5 06-Jul-2012 00:54:39

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @posTracker_OpeningFcn, ...
                   'gui_OutputFcn',  @posTracker_OutputFcn, ...
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

% --- Executes just before posTracker is made visible.
function posTracker_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to posTracker (see VARARGIN)

% Choose default command line output for posTracker
handles.output = hObject;
% Update handles structure
guidata(hObject, handles);

% UIWAIT makes posTracker wait for user response (see UIRESUME)
% uiwait(handles.figure1);
%init once 
handles=initOnce(handles);
guidata(hObject, handles);
%init camera
initCamera(handles);
%init Arduino
initArduino(handles);
%init data
handles=init(handles);
guidata(hObject, handles);

%detect cameras
function camDevices=detectCamera()
%determine PC or MAC OS
if ispc
    hwInfo = imaqhwinfo('winvideo');
elseif isunix
    hwInfo = imaqhwinfo('macvideo');
end
if ~isempty(hwInfo.DeviceInfo)
    camDevices=hwInfo.DeviceInfo;
    
else
    camDevices=[];
    msgbox('No camera found! Running without camera mode!',...
        'NO Camera','warn');
end

%init functions
function initCamera(handles)
camDevices=detectCamera();
setappdata(0,'camDevices',camDevices);
%load cameras to camList listbox
camName=cell(1,length(camDevices));
for i=1:length(camDevices)
    camName{i}=camDevices(i).DeviceName;
end
set(handles.listbox_camList,'String',camName);
set(handles.listbox_camList,'Value',1);
if ~isempty(camDevices)
    %vDefault='RGB24_320x240';
    vDefault='YUY2_320x240';
    camID=1;
    vFormats=camDevices(camID).SupportedFormats;
    vFnum=0;
    for i=1:length(vFormats)
        if strcmpi(vFormats{i},vDefault)
            vFnum=i;
        end
    end
    if vFnum>0
        vFmt=vDefault;
    else
        vFmt=camDevices(camID).DefaultFormat;
        vFnum=1;
    end
    %load all supported Formats to listbox
    set(handles.listbox_vFormats,'String',vFormats);
    set(handles.listbox_vFormats,'Value',vFnum);    
    
    vidobj = createVidObj(camDevices(camID),vFmt,handles);
    %init the camera-panel information
    initCamPanel(vidobj,handles);
else
    vidobj=[];
end
setappdata(0,'vidobj',vidobj);

%init the Arduino 
function initArduino(handles)
setappdata(0,'myArduino',[]);
port=instrhwinfo ('serial');
spNum=length(port.SerialPorts);
if spNum>0
    %load the list
    set(handles.listbox_Arduino_portNames,'String',port.SerialPorts);
    if spNum>0
        set(handles.listbox_Arduino_portNames,'Value',1);  
        portName=port.SerialPorts{1};
        setappdata(0,'serialPort',portName);
        try 
            myArduino=serial(portName,'BaudRate',9600);
            %set(myArduino,'Timeout',1);    
            setappdata(0,'myArduino',myArduino);
            fopen(myArduino);
            disp('Connect to Arduino, and initialize...');
            %pause(2);    
            %sendArduino(myArduino,'C','0','A');
            disp('Arduino connected successfully!');
        catch exception
            %h = msgbox('Fail to connect Arduino, please check serail port, then try again!',...
            %    'Arduino connection error','error'); 
            msgbox('No Arduino found, please check serial port and try again!',...
                'NO Arduino','warn');
            setappdata(0,'myArduino',[]);
            return;
        end
    end
else
    msgbox('No serial port found!','No Port','warn');
    return;
end


%init only once
function handles=initOnce(handles)
versionName='posTrack_v1.83';
set(handles.figure1,'Name',versionName);
setappdata(0,'versionName',versionName);
setappdata(0,'PathName','C:\2Chambers\');
%setappdata(0,'PathName','D:\');
setappdata(0,'recTime',1800);      %total record time
set(handles.edit_RecTime,'String',s2hhmmss(1800,0));
threshold=100;          %signal/noise ratio to detect lick/flash
set(handles.edit_Threshold,'String',num2str(threshold));
setappdata(0,'threshold',threshold);
setappdata(0,'VideoSaveTag',1);
set(handles.checkbox_VideosaveTag,'value',1);
setappdata(0,'OnlineTag',1);
set(handles.checkbox_OnlineTag,'value',1);
setappdata(0,'mplayerTag',0);
set(handles.checkbox_MplayerTag,'value',0);
conditionedArea=1;
setappdata(0,'conditionedArea',conditionedArea);
set(handles.edit_conditionedAreaNum,'String',num2str(conditionedArea));
%for camera panel
handles.camZoom=0;
set(handles.slider_Camera_Zoom,'Min',0','Max',10,'sliderstep',[0.1,0.1]);
set(handles.slider_Camera_Zoom,'value',handles.camZoom);
set(handles.edit_Camera_Zoom,'String',num2str(handles.camZoom));
handles.camPan=0;
set(handles.slider_Camera_Pan,'Min',-50','Max',50,'sliderstep',[0.01,0.01]);
set(handles.slider_Camera_Pan,'value',handles.camPan);
set(handles.edit_Camera_Pan,'String',num2str(handles.camPan));
handles.camTilt=0;
set(handles.slider_Camera_Tilt,'Min',-50','Max',50,'sliderstep',[0.01,0.01]);
set(handles.slider_Camera_Tilt,'value',handles.camTilt);
set(handles.edit_Camera_Tilt,'String',num2str(handles.camTilt));


function handles=init(handles)        
setappdata(0,'ctrlState',0);            %control-state: 0/none;1/preview;2/record;3/load;4/analyze
setappdata(0,'ctrlRecordTag',0);        %ctrl-record Tag:1/record;0/stop
setappdata(0,'ctrlPreviewTag',0);       %ctrl-preview Tag:1/preview;0/stop
setappdata(0,'ctrlROIShowTag',0);
ROI=struct('x',[],'y',[],'width',[],'height',[],'handle',[]);
setappdata(0,'ROI',ROI);
setappdata(0,'nowFrame',[]);        %current frame (displayed) image
setappdata(0,'aviobj',[]);
setappdata(0,'totalTim',0);
setappdata(0,'nowTim',0);       %current time(since record-on)
setappdata(0,'chamberMap',[]);      %map of chambers
setappdata(0,'hfig',[]);
setappdata(0,'backgroundImg',[]);
setappdata(0,'bkImggray',[]);
initData();

%init the data
function initData()
setappdata(0,'trackData',[]);       %track data = [time,x,y];
setappdata(0,'scoreData',[]);
setappdata(0,'binData',[]);
setappdata(0,'freezeTim',0);         %freezing time in photo-chamber
global prePosHandle;
prePosHandle=[];
%init the hardware
myArduino=getappdata(0,'myArduino');
if ~isempty(myArduino)  
    %laser off
    fwrite(myArduino,'0','uchar');
end


%create video object
function vidobj=createVidObj(camDevice,vFormat,handles)
if ispc
    vidobj = videoinput('winvideo',camDevice.DeviceID,vFormat);
elseif isunix
    vidobj = videoinput('macvideo',camDevice.DeviceID,vFormat);
end
set(vidobj,'ReturnedColorSpace','rgb');
%init important functions here
set(vidobj,'StartFcn',{@myStartCam,handles});
set(vidobj,'StopFcn',{@myStopCam,handles});
set(vidobj,'TimerPeriod',0.3);
set(vidobj,'TimerFcn',{@myTimerCam,handles});
set(vidobj,'FramesPerTrigger',inf);
set(vidobj,'TriggerRepeat',0); 
set(vidobj,'FramesAcquiredFcnCount',10);
set(vidobj,'FramesAcquiredFcn',{@myFrames_AfterAcquired,handles});

    
%init the camera pannel 
function initCamPanel(vidobj,handles)
src = getselectedsource(vidobj);   
srcInfo=get(src);
%set(src,'FocusMode','auto');
%set(src,'ExposureMode','auto');
if isfield(srcInfo,'FrameRate')
       src.FrameRate='30';
end
% if isfield(srcInfo,'WhiteBalance')
%         set(src,'WhiteBalanceMode','manual');
%         src.WhiteBalance = 2900;
% end
if isfield(srcInfo,'Zoom')
        src.zoom = 0;
        set(handles.edit_Camera_Zoom,'Enable','on');
        set(handles.slider_Camera_Zoom,'Enable','on');    
else
        set(handles.edit_Camera_Zoom,'Enable','off');
        set(handles.slider_Camera_Zoom,'Enable','off');
end
if isfield(srcInfo,'Tilt')
        src.Tilt = 0;
        set(handles.edit_Camera_Tilt,'Enable','on');
        set(handles.slider_Camera_Tilt,'Enable','on');    
else
        set(handles.edit_Camera_Tilt,'Enable','off');
        set(handles.slider_Camera_Tilt,'Enable','off');
end
if isfield(srcInfo,'Pan')
        src.Pan = 0;
        set(handles.edit_Camera_Tilt,'Enable','on');
        set(handles.slider_Camera_Tilt,'Enable','on');    
else
        set(handles.edit_Camera_Pan,'Enable','off');
        set(handles.slider_Camera_Pan,'Enable','off');
end


% --- Outputs from this function are returned to the command line.
function varargout = posTracker_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;

% --- Executes on button press in ctrl_Preview.
function ctrl_Preview_Callback(hObject, eventdata, handles)
% hObject    handle to ctrl_Preview (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
vidobj=getappdata(0,'vidobj');
ctrlPreviewTag=getappdata(0,'ctrlPreviewTag');
if ctrlPreviewTag==1
    closepreview(vidobj);
    set(hObject,'String','Preview');
    ctrlPreviewTag=0;
    ctrlState=0;        
    %F=getframe;
    %nowFrame=F.cdata;
else        
    set(hObject,'String','Stop');
    ctrlPreviewTag=1;
    ctrlState=1;    
    preview(vidobj);
end
setappdata(0,'ctrlState',ctrlState);
setappdata(0,'ctrlPreviewTag',ctrlPreviewTag);
guidata(hObject,handles);

        
% --- Executes on button press in ctrl_Record.
function ctrl_Record_Callback(hObject, eventdata, handles)
% hObject    handle to ctrl_Record (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
VideoSaveTag=getappdata(0,'VideoSaveTag');
vidobj=getappdata(0,'vidobj');
frameNums=getappdata(0,'frameNums');
recTime=getappdata(0,'recTime');
ctrlRecordTag=getappdata(0,'ctrlRecordTag');
src = getselectedsource(vidobj);
srcInfo = get(src);
if ctrlRecordTag==1   
    stop(vidobj);
    if isfield(srcInfo,'FocusMode')
        set(src,'FocusMode','auto'); 
    end
else        
    set(hObject,'String','Stop');
    OnlineTag=getappdata(0,'OnlineTag');
    ROI=getappdata(0,'ROI');
    backgroundImg=getappdata(0,'backgroundImg');
    if isempty(backgroundImg) 
        if OnlineTag
            %msgbox('No background image, On-line processing off!','Warning','warn');
            set(handles.checkbox_OnlineTag,'value',0);
            setappdata(0,'OnlineTag',0);
        end
    else
        %clear the figure
        showBackground_ROI(backgroundImg,ROI,handles);
    end
    
    if isfield(srcInfo,'FocusMode')
        set(src,'FocusMode','manual');
    end
    frameNums(1:2)=[0,0];
    if isfield(srcInfo,'FrameRate')
        frameNums(3)=str2double(src.FrameRate);
    else
        frameNums(3)=30;
    end
    setappdata(0,'frameNums',frameNums);
    setappdata(0,'ctrlRecordTag',1);
    setappdata(0,'ctrlState',2);
    if recTime==0
        set(vidobj, 'FramesPerTrigger', inf);
    else
        set(vidobj, 'FramesPerTrigger', recTime*30);
    end
    if VideoSaveTag
        set(vidobj, 'LoggingMode', 'Disk&memory');
        FileName=getFileName('.avi');
        setappdata(0,'FileName',FileName);
        PathName=getappdata(0,'PathName');
        fname=strcat(PathName,FileName);
        aviobj=VideoWriter(fname);
        vidobj.DiskLogger = aviobj;
    else
        set(vidobj, 'LoggingMode', 'memory');
    end
    initData();
    start(vidobj);    
end
guidata(hObject,handles);

%To show the background Image and ROI
function showBackground_ROI(backgroundImg,ROI,handles)
[vHei,vWid,chs]=size(backgroundImg);
hfig=getappdata(0,'hfig');
hfig=creatFigure(hfig,vWid,vHei);
setappdata(0,'hfig',hfig);
imagesc(backgroundImg);
axis off;
hold on;
%draw the ROI
n=length(ROI.x);
if n>0
    cls=get(gca,'colororder');
	for i=1:n
        h=rectangle('Position',[ROI.x(i),ROI.y(i),ROI.width(i),ROI.height(i)],...
                'LineStyle','--','edgecolor',cls(i+1,:));
        ROI.handle(i)=h;    
    end
    setappdata(0,'ROI',ROI);
end    

%get handle to figure
function handle=creatFigure(hfig,vWid,vHei)
if ishandle(hfig)
	handle=figure(hfig);
    cla;
else       
    handle=figure('position',[100,100,vWid,vHei]);  
	set(gca,'position',[0,0,1,1]);
	axis image off;
end


%save the data to .mat
function saveMatFile()
FileName=getappdata(0,'FileName'); 
PathName=getappdata(0,'PathName');
ROI=getappdata(0,'ROI');
trackData=getappdata(0,'trackData');
backgroundImg=getappdata(0,'backgroundImg');
chamberMap=getappdata(0,'chamberMap');
scoreData=getappdata(0,'scoreData');
mfile=strcat(FileName(1:length(FileName)-4),'.mat');
filename=strcat(PathName,mfile);
save(filename,'filename','ROI','trackData','backgroundImg','chamberMap',...
    'scoreData');

%function when start(vidobj)
function myStartCam(hObject, eventdata,handles)
setappdata(0,'nowTim',0);  
tic;

%function when stop(vidobk)
function myStopCam(hObject, eventdata,handles)
vidobj=getappdata(0,'vidobj');    
%stoppreview(vidobj);
set(handles.ctrl_Record,'String','Record');
setappdata(0,'ctrlState',0);
setappdata(0,'ctrlRecordTag',0);
setappdata(0,'ctrlState',4);
OnlineTag=getappdata(0,'OnlineTag');
ROI=getappdata(0,'ROI');
if OnlineTag    
    trackData=getappdata(0,'trackData');
    backgroundImg=getappdata(0,'backgroundImg');
    %bkImg=rgb2gray(backgroundImg);
    chamberMap=getappdata(0,'chamberMap');
else
    frameNums=getappdata(0,'frameNums');
    frmRate=floor(frameNums(3)+0.1);
    nFrames=vidobj.FramesAvailable;
    trackData=zeros([],3); 
    totalTim=floor(nFrames/frmRate);
    %generate background images
    if totalTim>5
        bkfrms=frmRate*5;
    else
        bkfrms=5;
    end
    mov=peekdata(vidobj,bkfrms);
    backgroundImg=max(mov,[],4);
    bkImg=rgb2gray(backgroundImg);   
    %generate threshold  
    th=getappdata(0,'threshold');
    if th==0
        th=getThreshold(mov,bkImg);
        setappdata(0,'threshold',th);
        set(handles.edit_Threshold,'String',num2str(th));
    end
    h=waitbar(0,'Generate data from video,please wait...'); 
    for i=1:totalTim
    	mov=getdata(vidobj,frmRate);
        frms=floor([1,frmRate]+(i-1)*frmRate);
        trackData(frms(1):frms(2),2:3)=getPosData1(mov,bkImg,th); 
        waitbar(i/totalTim);   
    end
    close(h);
    trackData(:,1)=(1:length(trackData(:,2)))/frmRate;
    frameNums(1)=totalTim*frameNums(3);
    frameNums(2)=frameNums(1);  
    %generate chamberMap
    chamberMap=getchambers(ROI,bkImg);
    setappdata(0,'backgroundImg',backgroundImg);
    setappdata(0,'bkImggray',bkImg);
    %setappdata(0,'trackData',trackData);
    setappdata(0,'frameNums',frameNums);
    setappdata(0,'chamberMap',chamberMap);
end
%remove zeros(mouse not found)
idx=find(trackData(:,2)>0 | trackData(:,3)>0);
trackData=trackData(idx,:);
setappdata(0,'trackData',trackData);
%calculate the index (time spent in chambers)
[scoreData,binData,posHotMap]=getScore(chamberMap,trackData);
setappdata(0,'scoreData',scoreData);
setappdata(0,'binData',binData);
disp(scoreData);
%draw the mouse track
showTrack(trackData,scoreData,backgroundImg,ROI,handles);
%draw the hotmap of position(for long-term recording)
showPosHotMap(posHotMap,scoreData,ROI,handles);
showbinData(binData,handles);
%save the video
VideoSaveTag=getappdata(0,'VideoSaveTag');
if VideoSaveTag   
    wait(vidobj,5);
    %save the data to txt-file
    FileName=getappdata(0,'FileName'); 
    PathName=getappdata(0,'PathName');
    fn=strcat(FileName(1:length(FileName)-4),'.txt');
    fn=strcat(PathName,fn);
    saveTxtData(fn);
	%also save the data to .mat	
    saveMatFile(); 
end


%function when timer(for video)
function myTimerCam(hObject, eventdata,handles)
nowTim=toc;
set(handles.txt_Timer,'String',s2hhmmss(nowTim,0));
freezeTim=getappdata(0,'freezeTim');
myArduino=getappdata(0,'myArduino');
%for propotol control, from Arduino.m
if ~isempty(myArduino)
    trackData=getappdata(0,'trackData');
    chamberMap=getappdata(0,'chamberMap');
    conditionedArea=getappdata(0,'conditionedArea'); 
    x=0;y=0;
    if ~isempty(trackData)
        x=round(trackData(end,2));
        y=round(trackData(end,3));
    end
    if nowTim-freezeTim<=5
        if x>0 && y>0
                if chamberMap(y,x)==conditionedArea
                    fwrite(myArduino,'1','uchar');
                else
                    fwrite(myArduino,'0','uchar');
                    setappdata(0,'freezeTim',toc);
                end
        end
    elseif nowTim-freezeTim>5 && nowTim-freezeTim<=5+5
        fwrite(myArduino,'0','uchar');
        if x>0 && y>0
                if chamberMap(y,x)~=conditionedArea
                    setappdata(0,'freezeTim',toc);
                end
        end
    elseif nowTim-freezeTim>5+5        %stop 5s after 20s contineous stimulation
        setappdata(0,'freezeTim',toc);
    end
end



%function when certain frames acquired
function myFrames_AfterAcquired(hObject, eventdata,handles) 
OnlineTag=getappdata(0,'OnlineTag');
if OnlineTag
    vidobj=getappdata(0,'vidobj');
    nFrames=vidobj.FramesAvailable;
    if nFrames>0
        nowTim=getappdata(0,'nowTim');
        frameNums=getappdata(0,'frameNums');
        trackData=getappdata(0,'trackData');
        bkImg=getappdata(0,'bkImggray');
        th=getappdata(0,'threshold');
        vidData=peekdata(vidobj,nFrames);
        flushdata(vidobj);
        nowFrmIdx=frameNums(1)+(1:nFrames);
        %get trackData
        trackData(nowFrmIdx,2:3)=getPosData1(vidData,bkImg,th);
        trackData(nowFrmIdx,1)=nowFrmIdx/frameNums(3);
        frameNums(1)=frameNums(1)+nFrames;
        frameNums(2)=frameNums(1); 
        setappdata(0,'trackData',trackData);
        setappdata(0,'frameNums',frameNums);
        %show realtime figure
        ShowRunData(trackData(nowFrmIdx,2:3),handles);  
        %update the real capture framerate
        nTim=toc;
        rlRate=min(nFrames/(nTim-nowTim),frameNums(3));            %real-time change
        set(handles.cam_frmHz_Indicator,'Position',[12,66,15,50*rlRate/frameNums(3)]);
        set(handles.cam_frmHz_Indicator,'BackgroundColor',[round(1-rlRate/frameNums(3)+0.2),...
                round(rlRate/frameNums(3)-0.2),1-rlRate/frameNums(3)]);
        setappdata(0,'nowTim',nTim);
        clear vidData;
    end
end


%to show realtime figure
function ShowRunData(nowPos,handles)
%hfig=getappdata(0,'hfig');
%figure(hfig);
%hold on;
% plot(trackData(:,2),trackData(:,3),'o','MarkerSize',2,...
%     'MarkerEdgeColor','b','MarkerFaceColor','b');
%hold on;
global prePosHandle;
delete(prePosHandle);
prePosHandle=plot(nowPos(:,1),nowPos(:,2),'o','MarkerSize',2,...
    'MarkerEdgeColor','r','MarkerFaceColor','r');


% --- Executes on button press in ctrl_Load.
function ctrl_Load_Callback(hObject, eventdata, handles)
% hObject    handle to ctrl_Load (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
PathName=getappdata(0,'PathName');
ctrlState=getappdata(0,'ctrlState');
if ctrlState>=3
    handles=init(handles);
    guidata(hObject,handles);
end
ctrlState=getappdata(0,'ctrlState');
if ctrlState==0
    [FileName,pName] = uigetfile({
        '*.avi','Avi File(*.avi)';
        '*.wmv','Windows media File(*.wmv)';        
        '*.mov','Mac movie File(*.mov)'; 
        '*.txt','Text Data File(*.txt)';
        '*.*','all file(*.*)';
        },'Load vedio file',PathName);
    if FileName~=0
        PathName=pName;
        fname=strcat(PathName,FileName);
        setappdata(0,'PathName',PathName);
        setappdata(0,'FileName',FileName);
        if strcmpi(FileName(end-3:end),'.txt')
            %read all data
            [trackData,scoreData,bkgImg,ROI]=readTxtData(fname);
            setappdata(0,'trackData',trackData);
            setappdata(0,'scoreData',scoreData);
            setappdata(0,'backgroundImg',bkgImg);
            setappdata(0,'ROI',ROI);
            %recalcute the score and show figure, based on recTime
            recTime=getappdata(0,'recTime');
            if trackData(end,1)>recTime
                idx=find(trackData(:,1)<recTime);
                if ~isempty(idx)
                    trackData=trackData(idx,:);
                end
            end
            bkImg=rgb2gray(bkgImg);
            chamberMap=getchambers(ROI,bkImg);
            [scoreData,binData,posHotMap]=getScore(chamberMap,trackData);
            %plot the result figure 
            showTrack(trackData,scoreData,bkgImg,ROI,handles);
            %also hotmap?
            showPosHotMap(posHotMap,scoreData,ROI,handles);
            setappdata(0,'chamberMap',chamberMap);
            setappdata(0,'posHotMap',posHotMap);
            setappdata(0,'binData',binData);
            %also show the curve with bin=5min
            showbinData(binData,handles);
        else
            %aviobj=VideoReader(fname);
            aviobj=mmreader(fname);
            setappdata(0,'aviobj',aviobj);
            %get some basic information of vedio
            vWid=aviobj.Width;
            vHei=aviobj.Height;
            %chamberMap=zeros(vHei,vWid);        
            totalTim=aviobj.Duration;
            setappdata(0,'totalTim',totalTim);
            nFrames=aviobj.NumberOfFrames;
            frmRate=aviobj.FrameRate;  
            if isempty(nFrames)
                disp('Recounting the number of frames, please waiting...');
                lastFrame=read(aviobj,inf);
                nFrames=aviobj.NumberOfFrames;
                disp('Counting done!');
            end
            if frmRate>nFrames/totalTim
                frmRate=nFrames/totalTim;        
            end    
            %get the background image (flat,replace mouse with bright background)
            mov=read(aviobj,[1+67*frmRate frmRate*70]);
            backgroundImg=max(mov,[],4);
            bkImg=rgb2gray(backgroundImg);
            setappdata(0,'backgroundImg',backgroundImg);
            setappdata(0,'bkImggray',bkImg);
            th=getappdata(0,'threshold');          
            %generate threshold
            if th==0
                th=getThreshold(mov,bkImg);
                setappdata(0,'threshold',th);
                set(handles.edit_Threshold,'String',num2str(th));
            end

            %just load the first frame here
            %select the region(ROI),then load all frames
            hfig=creatFigure([],vWid,vHei);  
            set(hfig,'Name',FileName);
            nowFrmNum=1;        
            mov=read(aviobj,nowFrmNum);
            imagesc(mov);
            %set(h,'UIContextMenu',handles.videoMenu);
            %axis off;        
            nowFrame=mov;    
            %figure('position',[400,100,vWid,vHei]); 
            setappdata(0,'ctrlState',3);
            frameNums=[nowFrmNum,nFrames,frmRate];            
            %display some information
            set(handles.txt_Timer,'String',s2hhmmss(frameNums(1)/frameNums(3),0));
            setappdata(0,'nowFrame',nowFrame);
            setappdata(0,'frameNums',frameNums);   
            setappdata(0,'hfig',hfig);
            mplayerTag=getappdata(0,'mplayerTag');
            if mplayerTag
                %show the movie?
                implay(fname);
            end
        end
    end
end
guidata(hObject,handles);

%read txt-file (data saved)
function [trackData,scoreData,bkgImg,ROI]=readTxtData(fname)
%trackData=[];
%scoreData=[];
%bkgImg=[];
ROI=struct('x',[],'y',[],'width',[],'height',[],'handle',[]);
fid=fopen(fname);
tmpstr=textscan(fid,'%s %s',3);
tmpstr=textscan(fid,'%s %f',1);
dotnum=tmpstr{2}(1);
tmpstr=textscan(fid,'%s %f',1);
chambernum=tmpstr{2}(1);
tmpstr=textscan(fid,'%s',1);
scoreData=fscanf(fid,'%f',[1,chambernum]);
tmpstr=textscan(fid,'%s',1);
ROIDat=fscanf(fid,'%f',[4,chambernum+1])';
bkgImg=uint8(zeros(ROIDat(1,3),ROIDat(1,4),3))+255;
ROI.x=ROIDat(2:end,1);
ROI.y=ROIDat(2:end,2);
ROI.width=ROIDat(2:end,3);
ROI.height=ROIDat(2:end,4);
tmpstr=textscan(fid,'%s',1);
trackData=fscanf(fid,'%f',[3,dotnum])';


%to generate the thresold 
function th=getThreshold(mov,bkImg)
bkTmp=min(mov,[],4);
bkTmp=rgb2gray(bkTmp);
deltaM=bkImg-bkTmp;
th=mean2(deltaM)+std2(deltaM);
%th=0.5*mean2(deltaM)+0.5*max(deltaM(:));

% --- Executes on button press in ctrl_Analyze.
function ctrl_Analyze_Callback(hObject, eventdata, handles)
% hObject    handle to ctrl_Analyze (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
ROI=getappdata(0,'ROI');
recTime=getappdata(0,'recTime');
ctrlState=getappdata(0,'ctrlState');
aviobj=getappdata(0,'aviobj');
frameNums=getappdata(0,'frameNums');
frmRate=floor(frameNums(3)+0.1);
if ctrlState>=3 && ~isempty(aviobj)     
    totalTim=aviobj.Duration;   
    setappdata(0,'totalTim',totalTim);    
    if recTime>0 && recTime<floor(totalTim);
        loadTim=recTime;        
    else
        loadTim=floor(totalTim-1);
    end    
    trackData=zeros([],3);   
    th=getappdata(0,'threshold');
    backgroundImg=getappdata(0,'backgroundImg');
    bkImg=rgb2gray(backgroundImg);
    %find the mouse position
    %hfig=getappdata(0,'hfig');
    %if ishandle(hfig)
    %    figure(hfig);        
    %end   
    step=1;         %data generated every n frame
    h=waitbar(0,'Generate track from video,please wait...');  
    for k = 1:loadTim         % Read 1s at a time.  
        frms=floor([1,frmRate]+(k-1)*frmRate);
        mov=read(aviobj,frms);
        mov=mov(:,:,:,1:step:end);
        n=floor((frmRate-1)/step)+1;
        ki=floor([1,n]+(k-1)*n);
        %Method1: detect mouse by dark-pixel (rgb)
        %trackData(ki(1):ki(2),2:3)=getPosData0(mov);
        %method2: detect mouse by subtraction (gray)
        trackData(ki(1):ki(2),2:3)=getPosData1(mov,bkImg,th); 
        waitbar(k/loadTim,h);
    end    
    close(h);  
    trackData(:,1)=(1:length(trackData(:,2)))/(frmRate/step);
    %remove zeros(mouse not found)
    idx=find(trackData(:,2)>0 | trackData(:,3)>0);
    trackData=trackData(idx,:);
    setappdata(0,'trackData',trackData);
    setappdata(0,'ctrlState',4);
    %generate chamberMap
    chamberMap=getchambers(ROI,bkImg);
    %calculate the index (time spent in chambers)
    [scoreData,binData,posHotMap]=getScore(chamberMap,trackData);
    %draw the mouse track
    showTrack(trackData,scoreData,backgroundImg,ROI,handles);
    %show score
    disp(scoreData);
    %figure;
    %bar(scoreData);
    %show the hotmap of position (for long-term recording)
    showPosHotMap(posHotMap,scoreData,ROI,handles);
    showbinData(binData,handles);
end
guidata(hObject,handles);


%To get the mouse position 
function posData=getPosData0(frames)
%frames RGB-image
frmnum=size(frames,4);
posData=zeros(frmnum,2);
th=30;          %threshold for black detection
%consider to auto-threshold (imhist, motion-based detection?
for i=1:frmnum
    %find the black spot (whole mouse)
    idx1=(frames(:,:,1,i)<th & frames(:,:,1,i)>1);
    idx2=(frames(:,:,2,i)<th & frames(:,:,2,i)>1);
    idx3=(frames(:,:,3,i)<th & frames(:,:,3,i)>1);
    idxMap=(idx1 & idx2 & idx3);
    %figure;imagesc(idxMap);
    idxMap= bwareaopen(idxMap,40);
    %figure;
    %imagesc(idxMap);
    [x,y]=find(idxMap);
    %set the center as the position 
    posData(i,1)=mean(y);
    posData(i,2)=mean(x);
    %imagesc(frames(:,:,:,i);
    %hold on;
    %plot(posData(i,1),posData(i,2),'+r');
    %pause(0.001);
end

%get mouse position by subtraction
function posData=getPosData1(mov,bkImg,th)
%mov=rgb, bkImg=gray-image [H,W,frm]
[vH,vW,chs,frmnum]=size(mov);
frames=mov(:,:,1,:)*0.30+mov(:,:,2,:)*0.59+mov(:,:,3,:)*0.11;
frames=reshape(frames,vH,vW,frmnum);
posData=zeros(frmnum,2);
bk=repmat(bkImg,[1,1,frmnum]);
M=(bk-frames)>th;
M=bwareaopen(M,50);        %remove sparse noise
[x,yt]=find(M);
t=ceil(yt/vW);
y=yt-(t-1)*vW;
%set the center as the position 
for i=1:frmnum
    idx=find(t==i);
    if ~isempty(idx)
        xc=mean(y(idx));
        yc=mean(x(idx));
        if ~isnan(xc) && ~isnan(yc)
            posData(i,1:2)=[xc yc];
        end
    end
end
clear bk frames M;
clear t x y yt;


%get chamberMap from ROI
function chamberMap=getchambers(ROI,bkImg)
[vH,vW]=size(bkImg);
chamberMap=zeros(vH,vW);
num=length(ROI.x);
for i=1:num
    x=[ROI.x(i),ROI.x(i)+ROI.width(i),ROI.x(i)+ROI.width(i),ROI.x(i),ROI.x(i)];
    y=[ROI.y(i),ROI.y(i),ROI.y(i)+ROI.height(i),ROI.y(i)+ROI.height(i),ROI.y(i)];
    bk1=poly2mask(x,y,vH,vW);
    chamberMap=chamberMap+bk1*i;
end

%calculate the score (time spent in each chamber)
function [scoreData,binData,posHotMap]=getScore(chamberMap,trackData)
chamberNum=max(chamberMap(:));
scoreData=zeros(1,chamberNum);
posHotMap=0*chamberMap;
binTim=5*60;            %time bin for curve
dn=ceil(trackData(end,1)/binTim);
binData=zeros(dn,chamberNum);
ptnum=length(trackData);
for i=1:ptnum
    k=ceil(trackData(i,1)/binTim);
    x=floor(trackData(i,2));
    y=floor(trackData(i,3));
    if x>0 && y>0
        posHotMap(y,x)=posHotMap(y,x)+1;
        px=chamberMap(y,x);
        if px>0
            scoreData(px)=scoreData(px)+1;
            binData(k,px)=binData(k,px)+1;
        end
    end
end
scoreData=scoreData./sum(scoreData);
for i=1:dn
    binData(i,:)=binData(i,:)/sum(binData(i,:));
end
posHotMap=posHotMap./(max(posHotMap(:)));
%smooth the hotmap
h=fspecial('gaussian',7,1);
posHotMap=imfilter(posHotMap,h);
%figure;imagesc(posHotMap);


%to draw the track
function showTrack(trackData,scoreData,backgroundImg,ROI,handles)
whitebk=backgroundImg*0+255;
showBackground_ROI(whitebk,ROI);
hfig=getappdata(0,'hfig');
%if ishandle(hfig)
    figure(hfig);
%end
%show the scoreData
if ~isempty(ROI.x)
    n=length(ROI.x);
    w0=min(ROI.width);
    c1=[0.5,0,0];
    c2=[0.75,0.75,0.75];
    for i=1:n
        %horizontal
        rectangle('position',[ROI.x(i),ROI.y(i)-12,w0*scoreData(i)+0.01,10],...
            'EdgeColor',c1,'FaceColor', c1);
        text(ROI.x(i),ROI.y(i)-6,num2str(round(scoreData(i)*100)/100),'color',c2);        
        %or vertical
%          rectangle('position',[ROI.x(i)+ROI.width(i),ROI.y(i),10,w0*scoreData(i)],...
%             'EdgeColor',c1,'FaceColor', c1);       
        %text(ROI.x(i)+ROI.width(i),ROI.y(i),num2str(round(scoreData(i)*100)/100),'color',c2);
    end
end
hold all;
plot(trackData(:,2),trackData(:,3),'-k','LineWidth',1);
%plot(trackData(:,2),trackData(:,3),'o','MarkerSize',1,'MarkerEdgeColor','b');


%draw the hotmap of position
function showPosHotMap(posHotMap,scoreData,ROI,handles)
clim=[0,mean2(posHotMap)+2*std2(posHotMap)];
[h,w]=size(posHotMap);
figure('position',[200,100,w,h]);
set(gca,'position',[0,0,1,1]);
%ROI
n=length(ROI.x);
if n>0
    %hotmap for ROIs
    for i=1:n
        hotROI=posHotMap(ROI.y(i):ROI.y(i)+ROI.height(i),ROI.x(i):ROI.x(i)+ROI.width(i));        
        imagesc(ROI.x(i),ROI.y(i),hotROI,clim);
        hold on;
    end
    axis image off;
    axis([0,w,0,h]);
    colormap('jet');
    set(gca,'clim',[0,0.02])
    c1=[0.5,0.5,0.5];
    c2=[0.75,0.75,0.75];
    w0=min(ROI.width);
    %cls=get(gca,'colororder');
	for i=1:n
        %rectangle('Position',[ROI.x(i),ROI.y(i),ROI.width(i),ROI.height(i)],...
        %        'LineStyle','--','edgecolor',[1,1,1]); 
        rectangle('position',[ROI.x(i),ROI.y(i)-12,w0*scoreData(i)+0.01,10],...
            'EdgeColor',c1,'FaceColor', c1);
        text(ROI.x(i),ROI.y(i)-6,num2str(round(scoreData(i)*100)/100),'color',c2);             
    end
else
    imagesc(posHotMap,clim);
    colormap('jet');
    set(gca,'clim',[0,0.05])
end

function showbinData(binData,handles)
figure;
%plot(binData);
bar(binData,'stacked');
xlabel('Time(bin#)');
ylabel('Time (%) in each chamber per 5min');

% --- Executes on button press in ctrl_Pathname.
function ctrl_Pathname_Callback(hObject, eventdata, handles)
% hObject    handle to ctrl_Pathname (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
PathName=getappdata(0,'PathName');
PathName = uigetdir(PathName);
if ~strcmpi(PathName(length(PathName)),'\')
    PathName=strcat(PathName,'\');
end
setappdata(0,'PathName',PathName);


%converter the time-format
function timstr=s2hhmmss(tim,sTag)
ss=mod(tim,60);
if ~sTag
    ss=floor(ss);
end
mm=mod(floor(tim/60),60);
hh=floor(tim/60/60);
if ss<10
    s=['0',num2str(ss)];
else
    s=num2str(ss);
end
if mm<10
    m=['0',num2str(mm)];
else
    m=num2str(mm);
end
if hh<10
    h=['0',num2str(hh)];
else
    h=num2str(hh);
end
timstr=[h,':',m,':',s];

%converter the time-format
function tim=hhmmss2s(timstr)
if length(timstr)==8
    tim=str2double(timstr(1:2))*60*60+str2double(timstr(4:5))*60+str2double(timstr(7:8));
else
    tim=0;
end

%get filename by current-time
function FileName=getFileName(extName)
nowstr=datestr(now,31);
nowstr(strfind(nowstr,':'))='-';
FileName=strcat(nowstr,extName);

function edit_RecTime_Callback(hObject, eventdata, handles)
% hObject    handle to edit_RecTime (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit_RecTime as text
%        str2double(get(hObject,'String')) returns contents of edit_RecTime as a double
timS=get(hObject,'String');
recTime=hhmmss2s(timS);
setappdata(0,'recTime',recTime);

% --- Executes during object creation, after setting all properties.
function edit_RecTime_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_RecTime (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in ctrl_Snapshot.
function ctrl_Snapshot_Callback(hObject, eventdata, handles)
% hObject    handle to ctrl_Snapshot (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
ctrlState=getappdata(0,'ctrlState');
if ctrlState<=1  %only when none/preview states
     vidobj=getappdata(0,'vidobj');
     vRes = get(vidobj, 'VideoResolution'); 
%      src = getselectedsource(vidobj); 
%      srcInfo=get(src);
%      if isfield(srcInfo,'Exposure')
%          src.Exposure=-5;
%      end
%      if isfield(srcInfo,'Brightness')
%          src.Brightness=160;
%      end
     snapshot = getsnapshot(vidobj);  
     hfig=getappdata(0,'hfig');
     hfig=creatFigure(hfig,vRes(1),vRes(2));
     imagesc(snapshot);
     nowFrame=snapshot;
     setappdata(0,'nowFrame',nowFrame);
     setappdata(0,'backgroundImg',snapshot);
     setappdata(0,'bkImggray',rgb2gray(snapshot));
     setappdata(0,'hfig',hfig);
end


% --- Executes on button press in ctrl_Clear.
function ctrl_Clear_Callback(hObject, eventdata, handles)
% hObject    handle to ctrl_Clear (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
%clear ROI-panel
set(handles.listbox_ROI,'String',[]);
set(handles.text_ROI_areaA,'BackgroundColor',[0.9,0.9,0.9]);
set(handles.text_ROI_areaB,'BackgroundColor',[0.9,0.9,0.9]);
hfig=getappdata(0,'hfig');
if ishandle(hfig)
    delete(hfig);
end
init(handles);


function edit_Threshold_Callback(hObject, eventdata, handles)
% hObject    handle to edit_Threshold (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit_Threshold as text
%        str2double(get(hObject,'String')) returns contents of edit_Threshold
%        as a double
threshold=str2double(get(hObject,'String'));
setappdata(0,'threshold',threshold);
mROIData=getappdata(0,'mROIData');
ctrlState=getappdata(0,'ctrlState');
if ctrlState==4 && ~isempty(mROIData)
    getDataShow(mROIData,handles);
end
 

% --- Executes during object creation, after setting all properties.
function edit_Threshold_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_Threshold (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on slider movement.
function slider_Camera_Zoom_Callback(hObject, eventdata, handles)
% hObject    handle to slider_Camera_Zoom (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of
%        slider
handles.camZoom=get(hObject,'Value');
set(handles.edit_Camera_Zoom,'String',num2str(handles.camZoom));
vidobj=getappdata(0,'vidobj');
src = getselectedsource(vidobj);    
src.Zoom = handles.camZoom;
guidata(hObject,handles);

% --- Executes during object creation, after setting all properties.
function slider_Camera_Zoom_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider_Camera_Zoom (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end

function edit_Camera_Zoom_Callback(hObject, eventdata, handles)
% hObject    handle to edit_Camera_Zoom (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit_Camera_Zoom as text
%        str2double(get(hObject,'String')) returns contents of
%        edit_Camera_Zoom as a double
handles.camZoom=str2double(get(hObject,'String'));
set(handles.slider_Camera_Zoom,'value',handles.camZoom);
vidobj=getappdata(0,'vidobj');
src = getselectedsource(vidobj);    
src.Zoom = handles.camZoom;
guidata(hObject,handles);

% --- Executes during object creation, after setting all properties.
function edit_Camera_Zoom_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_Camera_Zoom (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

% --- Executes on slider movement.
function slider_Camera_Pan_Callback(hObject, eventdata, handles)
% hObject    handle to slider_Camera_Pan (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
handles.camPan=get(hObject,'Value');
set(handles.edit_Camera_Pan,'String',num2str(handles.camPan));
vidobj=getappdata(0,'vidobj');
src = getselectedsource(vidobj);    
src.Pan = handles.camPan;
guidata(hObject,handles);

% --- Executes during object creation, after setting all properties.
function slider_Camera_Pan_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider_Camera_Pan (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end

function edit_Camera_Pan_Callback(hObject, eventdata, handles)
% hObject    handle to edit_Camera_Pan (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit_Camera_Pan as text
%        str2double(get(hObject,'String')) returns contents of edit_Camera_Pan as a double
handles.camPan=str2double(get(hObject,'String'));
set(handles.slider_Camera_Pan,'value',handles.camPan);
vidobj=getappdata(0,'vidobj');
src = getselectedsource(vidobj);    
src.Pan = handles.camPan;
guidata(hObject,handles);

% --- Executes during object creation, after setting all properties.
function edit_Camera_Pan_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_Camera_Pan (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

% --- Executes on slider movement.
function slider_Camera_Tilt_Callback(hObject, eventdata, handles)
% hObject    handle to slider_Camera_Tilt (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
handles.camTilt=get(hObject,'Value');
set(handles.edit_Camera_Tilt,'String',num2str(handles.camTilt));
vidobj=getappdata(0,'vidobj');
src = getselectedsource(vidobj);    
src.Tilt = handles.camTilt;
guidata(hObject,handles);

% --- Executes during object creation, after setting all properties.
function slider_Camera_Tilt_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider_Camera_Tilt (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end

function edit_Camera_Tilt_Callback(hObject, eventdata, handles)
% hObject    handle to edit_Camera_Tilt (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit_Camera_Tilt as text
%        str2double(get(hObject,'String')) returns contents of edit_Camera_Tilt as a double
handles.camTilt=str2double(get(hObject,'String'));
set(handles.slider_Camera_Tilt,'value',handles.camTilt);
vidobj=getappdata(0,'vidobj');
src = getselectedsource(vidobj);    
src.Tilt = handles.camTilt;
guidata(hObject,handles);

% --- Executes during object creation, after setting all properties.
function edit_Camera_Tilt_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_Camera_Tilt (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

% --- Executes on selection change in listbox_vFormats.
function listbox_vFormats_Callback(hObject, eventdata, handles)
% hObject    handle to listbox_vFormats (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns listbox_vFormats contents as cell array
%        contents{get(hObject,'Value')} returns selected item from listbox_vFormats
vFormats=cellstr(get(hObject,'String'));
vnum=get(hObject,'Value');
vidobj=getappdata(0,'vidobj');
if ~isempty(vidobj)
    delete(vidobj);
end
camDevices=getappdata(0,'camDevices');
camID=get(handles.listbox_camList,'Value');
vidobj = createVidObj(camDevices(camID),vFormats{vnum},handles);
setappdata(0,'vidobj',vidobj);
%guidata(hObject, handles);


% --- Executes during object creation, after setting all properties.
function listbox_vFormats_CreateFcn(hObject, eventdata, handles)
% hObject    handle to listbox_vFormats (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: listbox controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes during object deletion, before destroying properties.
function figure1_DeleteFcn(hObject, eventdata, handles)
% hObject    handle to figure1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
%clear all-data from memory
rmappdata(0,'trackData');
rmappdata(0,'backgroundImg');
rmappdata(0,'bkImggray');
rmappdata(0,'chamberMap');
rmappdata(0,'scoreData');

%clear the camera
vidobj=getappdata(0,'vidobj');
delete(vidobj);
rmappdata(0,'vidobj');

%Arduino
myArduino=getappdata(0,'myArduino');
if ~isempty(myArduino)
    fclose(myArduino);
    delete(myArduino);
    rmappdata(0,'myArduino');
end



% --- Executes on button press in checkbox_VideosaveTag.
function checkbox_VideosaveTag_Callback(hObject, eventdata, handles)
% hObject    handle to checkbox_VideosaveTag (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of checkbox_VideosaveTag
VideoSaveTag=get(hObject,'Value');
setappdata(0,'VideoSaveTag',VideoSaveTag);

% --- Executes on button press in checkbox_OnlineTag.
function checkbox_OnlineTag_Callback(hObject, eventdata, handles)
% hObject    handle to checkbox_OnlineTag (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% Hint: get(hObject,'Value') returns toggle state of checkbox_OnlineTag
OnlineTag=get(hObject,'Value');
setappdata(0,'OnlineTag',OnlineTag);


% --- Executes on selection change in listbox_Arduino_portNames.
function listbox_Arduino_portNames_Callback(hObject, eventdata, handles)
% hObject    handle to listbox_Arduino_portNames (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% Hints: contents = cellstr(get(hObject,'String')) returns listbox_Arduino_portNames contents as cell array
%        contents{get(hObject,'Value')} returns selected item from listbox_Arduino_portNames
list=cellstr(get(hObject,'String'));
num=get(hObject,'Value');
setappdata(0,'serialPort',list{num});


% --- Executes during object creation, after setting all properties.
function listbox_Arduino_portNames_CreateFcn(hObject, eventdata, handles)
% hObject    handle to listbox_Arduino_portNames (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: listbox controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in pushbutton_Arduino_Connect.
function pushbutton_Arduino_Connect_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton_Arduino_Connect (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
serialPort=getappdata(0,'serialPort');
myArduino=serial(serialPort,'BaudRate',9600);
%open the port
fopen(myArduino);
setappdata(0,'myArduino',myArduino);

% --- Executes on button press in pushbutton_Arduino_Disconnect.
function pushbutton_Arduino_Disconnect_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton_Arduino_Disconnect (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
myArduino=getappdata(0,'myArduino');
if ~isempty(myArduino)
    fclose(myArduino);
    delete(myArduino);
    setappdata(0,'myArduino',[]);
end

% --- Executes on button press in checkbox_MplayerTag.
function checkbox_MplayerTag_Callback(hObject, eventdata, handles)
% hObject    handle to checkbox_MplayerTag (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% Hint: get(hObject,'Value') returns toggle state of checkbox_MplayerTag
val=get(hObject,'Value');
setappdata(0,'mplayerTag',val);
if val 
    PathName=getappdata(0,'PathName');
    FileName=getappdata(0,'FileName');
    fname=strcat(PathName,FileName);
    implay(fname);
end


% --- Executes on button press in pushbutton_ROI_Add.
function pushbutton_ROI_Add_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton_ROI_Add (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
nowFrame=getappdata(0,'nowFrame');
ctrlState=getappdata(0,'ctrlState');
if ctrlState>=3 || ~isempty(nowFrame)  
    hfig=getappdata(0,'hfig');
    if ishandle(hfig)
        figure(hfig);
    end
    cls=get(gca,'colororder');
    ROI=getappdata(0,'ROI');
    n=length(ROI.x)+1;
    %set(handles.figure1,'pointer','crosshair');
    [x,y] = ginput(2);
    x1=min(x);
    y1=min(y);
    w=abs(x(2)-x(1));
    h=abs(y(2)-y(1));
    handle=rectangle('Position',[x1,y1,w,h],'LineStyle','--','edgecolor',cls(n+1,:));
    ROI=struct('x',[ROI.x x1],'y',[ROI.y y1],'width',[ROI.width w],'height',[ROI.height h],...
        'handle',[ROI.handle handle]);   
    setappdata(0,'ROI',ROI);
    %add the listbox
    listname=cell(1,n);
    for i=1:n
        listname{i}=strcat('Area',num2str(i));
    end
    set(handles.listbox_ROI,'String',listname);
    set(handles.listbox_ROI,'Value',n);
    %update the color label    
    if n==1
        set(handles.text_ROI_areaA,'BackgroundColor',cls(n+1,:));
    elseif n==2
        set(handles.text_ROI_areaB,'BackgroundColor',cls(n+1,:));
    end
    %get chamberMap for online-analysis
    backgroudImg=getappdata(0,'backgroundImg');
    bkImg=rgb2gray(backgroudImg);
    chamberMap=getchambers(ROI,bkImg);
    setappdata(0,'chamberMap',chamberMap);
    setappdata(0,'ctrlROIShowTag',1);
    set(handles.pushbutton_ROI_ShowHide,'String','Hide');
end


% --- Executes on button press in pushbutton_ROI_Remove.
function pushbutton_ROI_Remove_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton_ROI_Remove (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
%remove from listbox
listname=cellstr(get(handles.listbox_ROI,'String'));
val=get(handles.listbox_ROI,'Value');
listnum=length(listname);
idx=find((1:listnum)~=val);
if isempty(idx)
    set(handles.listbox_ROI,'String',[])
else
    listname=listname(idx);
    set(handles.listbox_ROI,'String',listname)
    set(handles.listbox_ROI,'Value',listnum-1)
end

%remove from ROI
ROI=getappdata(0,'ROI');
if ~isempty(ROI.handle)
    delete(ROI.handle(val));
end
ROI=struct('x',ROI.x(idx),'y',ROI.y(idx),'width',ROI.width(idx),...
    'height',ROI.height(idx),'handle',ROI.handle(idx));
setappdata(0,'ROI',ROI);

%remov the color label
if val==1
	set(handles.text_ROI_areaA,'BackgroundColor',[0.9,0.9,0.9]);
elseif val==2
	set(handles.text_ROI_areaB,'BackgroundColor',[0.9,0.9,0.9]);
end

% --- Executes on selection change in listbox_ROI.
function listbox_ROI_Callback(hObject, eventdata, handles)
% hObject    handle to listbox_ROI (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns listbox_ROI contents as cell array
%        contents{get(hObject,'Value')} returns selected item from listbox_ROI


% --- Executes during object creation, after setting all properties.
function listbox_ROI_CreateFcn(hObject, eventdata, handles)
% hObject    handle to listbox_ROI (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: listbox controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in pushbutton_ROI_ShowHide.
function pushbutton_ROI_ShowHide_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton_ROI_ShowHide (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
ctrlROIShowTag=getappdata(0,'ctrlROIShowTag');
ctrlROIShowTag=mod(ctrlROIShowTag+1,2);
setappdata(0,'ctrlROIShowTag',ctrlROIShowTag);
ROI=getappdata(0,'ROI');
rn=length(ROI.x);
hfig=getappdata(0,'hfig');
if ishandle(hfig)
    figure(hfig);
end
if ctrlROIShowTag
    cls=get(gca,'colororder');
    set(handles.pushbutton_ROI_ShowHide,'String','Hide');
    for i=1:rn
        h=rectangle('Position',[ROI.x(i),ROI.y(i),ROI.width(i),ROI.height(i)],...
            'LineStyle','--','edgecolor',cls(i+1,:));
        ROI.handle(i)=h;
    end
else
    set(handles.pushbutton_ROI_ShowHide,'String','Show');
    for i=rn:-1:1
        delete(ROI.handle(i));
        ROI.handle(i)=[];
    end
end
setappdata(0,'ROI',ROI);


% --- Executes on button press in pushbutton_ROI_Histgram.
function pushbutton_ROI_Histgram_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton_ROI_Histgram (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
ROI=getappdata(0,'ROI');
%I=getappdata(0,'backgroundImg');
I=getappdata(0,'nowFrame');
n=length(ROI.x);
if n>0
    bkImg=rgb2gray(I);
    for i=1:n
        M=bkImg(ROI.y(i):ROI.y(i)+ROI.height(i),ROI.x(i):ROI.x(i)+ROI.width(i));
        tname=strcat('Area',num2str(i));
        figure('Numbertitle','off','Name',tname);
        imhist(M);
    end
end


% --- Executes on selection change in listbox_camList.
function listbox_camList_Callback(hObject, eventdata, handles)
% hObject    handle to listbox_camList (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns listbox_camList contents as cell array
%        contents{get(hObject,'Value')} returns selected item from listbox_camList
camDevices=getappdata(0,'camDevices');
cnum=get(hObject,'Value');
%update the vFormat listbox
vFormats=camDevices(cnum).SupportedFormats;
set(handles.listbox_vFormats,'String',vFormats);
set(handles.listbox_vFormats,'Value',1);  
%create vidobj using defaultFormat
vidobj=getappdata(0,'vidobj');
if ~isempty(vidobj)
    delete(vidobj);
end
vidobj = createVidObj(camDevices(cnum),camDevices(cnum).DefaultFormat,handles);
setappdata(0,'vidobj',vidobj);



% --- Executes during object creation, after setting all properties.
function listbox_camList_CreateFcn(hObject, eventdata, handles)
% hObject    handle to listbox_camList (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: listbox controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in pushbutton_ctrl_txtData.
function pushbutton_ctrl_txtData_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton_ctrl_txtData (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
PathName=getappdata(0,'PathName');
[FileName,PName] = uiputfile('*.txt','Save track data',PathName);
if FileName~=0
    PathName=PName;
    setappdata(0,'PathName',PathName);
    fn=strcat(PathName,FileName);
    saveTxtData(fn);
end

function saveTxtData(FileName)
versionName=getappdata(0,'versionName');
conditionedArea=getappdata(0,'conditionedArea');
nowTim=getappdata(0,'nowTim');
ROI=getappdata(0,'ROI');
bkImg=getappdata(0,'bkImggray');
wh=size(bkImg);
trackData=getappdata(0,'trackData');
scoreData=getappdata(0,'scoreData');
cnum=length(scoreData);
fid=fopen(FileName,'wt');
fprintf(fid,'Code_version: %s\r\n',versionName);
fprintf(fid,'Chamber_coupled_with_reinforcement: %d\r\n',conditionedArea);
fprintf(fid,'Total_recorded_time(s): %d\r\n',floor(nowTim));
fprintf(fid,'Total_tracked_dots: %d\r\n',size(trackData,1));
fprintf(fid,'Number_of_chambers: %d\r\n',cnum);
fprintf(fid,'Score_for_chambers(1/2):');
fprintf(fid,'%6.2f ',scoreData);
fprintf(fid,'\r\n\r\n');
fprintf(fid,'ROI_of_chambers:\r\n');
%first: resolution of background image
fprintf(fid,'%d %d %d %d\r\n',0,0,wh(1),wh(2));
for i=1:cnum
    fprintf(fid,'%d %d %d %d\r\n',ROI.x(i),ROI.y(i),ROI.width(i),ROI.height(i));
end
fprintf(fid,'\r\n');
fprintf(fid,'Track_points(time/x/y):\r\n');
fprintf(fid,'%8.2f %8.2f %8.2f\r\n',trackData');
fclose(fid);



function edit_conditionedAreaNum_Callback(hObject, eventdata, handles)
% hObject    handle to edit_conditionedAreaNum (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit_conditionedAreaNum as text
%        str2double(get(hObject,'String')) returns contents of edit_conditionedAreaNum as a double
cnum=str2double(get(hObject,'String'));
setappdata(0,'conditionedArea',cnum);

% --- Executes during object creation, after setting all properties.
function edit_conditionedAreaNum_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_conditionedAreaNum (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
