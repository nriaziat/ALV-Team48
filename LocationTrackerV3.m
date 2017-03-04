
function varargout = LocationTrackerV3(varargin)
% LOCATIONTRACKERV3 MATLAB code for LocationTrackerV3.fig
%      LOCATIONTRACKERV3, by itself, creates a new LOCATIONTRACKERV3 or raises the existing
%      singleton*.
%
%      H = LOCATIONTRACKERV3 returns the handle to a new LOCATIONTRACKERV3 or the handle to
%      the existing singleton*.
%
%      LOCATIONTRACKERV3('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in LOCATIONTRACKERV3.M with the given input arguments.
%
%      LOCATIONTRACKERV3('Property','Value',...) creates a new LOCATIONTRACKERV3 or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before LocationTrackerV3_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to LocationTrackerV3_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help LocationTrackerV3

% Last Modified by GUIDE v2.5 07-Mar-2016 12:29:52

% Begin initialization code - DO NOT EDIT
    gui_Singleton = 1;
    gui_State = struct('gui_Name',       mfilename, ...
                       'gui_Singleton',  gui_Singleton, ...
                       'gui_OpeningFcn', @LocationTrackerV3_OpeningFcn, ...
                       'gui_OutputFcn',  @LocationTrackerV3_OutputFcn, ...
                       'gui_CloseRequestFcn', @LocationTrackerStudentv2_CloseRequestFcn, ...
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
end
% End initialization code - DO NOT EDIT

% --- Executes just before LocationTrackerV3 is made visible.
function LocationTrackerV3_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to LocationTrackerV3 (see VARARGIN)

% Choose default command line output for LocationTrackerV3
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

clc;
%% Establish Bluetooth Connection

% run loop until team name correctly entered
while (1)
    name = input('Enter the name of the NXT (TeamXX): ','s');
    % use short circuit to test strings of correct length only
    if ((length(name) == 6) && (strcmp(name(1:4), 'Team') == 1))
       teamNum = uint16(str2double(name(5:6)));
       if (teamNum > 0) && (teamNum < 71)
           break;
       end
    end
    disp('Try again!'); 
end

% establish initial bluetooth connection
nxt = NXTInit(name);

% Beginning the plot of manual/automatic
horizontal = 5; % Number of boxes in horizontal direction
vertical = 5; % Number of boxes in vertical direction
dim_w = 150; % Box width, mm
dim_h = 150; % Box height, mm

% versions = 'student', 'PoC', 'Final'
version = 'student';

axes(handles.axes1);

finalVersion = false;
if strcmp(version, 'PoC') == 1
    loadObstMapPoC
elseif strcmp(version, 'Final') == 1
    disp('Loading camera')
    loadSetupFinal
    disp('System initialized fully')
    finalVersion = true;
elseif strcmp(version, 'student') == 1
    obstacle = zeros(vertical, horizontal);
    obstacle(1, 2) = 1;
else
    disp('Version not specified correctly');
end

% horizontal, vertical, dim_w, and dim_h updated in loadSetupFinal
max_x = horizontal * dim_w;
max_y = vertical * dim_h;

%% initial setup completed, start loop for UserClick
while (1)
    cla reset; % clear objects from memory all the time
    
    % exit if GUI is closed
    if ~ishandle(hObject)
        fclose(nxt);
        break;
    end

    % use for system check

    % Plotting the board on the figure, common to both automatic and manual
    hold on;
        
    axis([0 max_x 0 max_y]);
    axis off;

    % Plot the vertical lines
    for i = 0:horizontal
        x = [(dim_w * i); (dim_w * i)];
        y = [0; max_y];
        p= plot(x, y); 
        set(p,'linewidth',3,'color',[0.5,0,0]); 
    end

    % Plot the horizontal lines
    for j = 0:vertical
        y = [(dim_h * j); (dim_h * j)];
        x = [0; max_x];
        p= plot(x, y); 
        set(p,'linewidth',3,'color',[0.5,0,0]); 
    end
    
    % Black out the obstacle locations
    for (i = 1:horizontal)
        for (j = 1:vertical) % This runs from top to bottom of map
            if obstacle(j,i) == 1
                yVals = [dim_h*(vertical-j), dim_h*(vertical-j), ...
                    dim_h*(vertical-j+1), dim_h*(vertical-j+1)];
                xVals = [dim_w*(i-1), dim_w*i, dim_w*i, dim_w*(i-1)];
                fill(xVals, yVals, 'k');
            end
        end
    end

    % Print the reference text, reference letters over matrix
    L1 = ['A', 'B', 'C', 'D', 'E', 'F', 'G', 'H', 'I', 'J', 'K', 'L', 'M', ...
        'N', 'O', 'P', 'Q', 'R', 'S', 'T', 'U', 'V', 'W', 'X', 'Y', 'Z'];
    for (i = 1:horizontal)
        for (j = 1:vertical) % This runs from top to bottom of map
            if (mod((i-1), 3) == 0) % 1, 4, 7, etc.
                if (mod((j-1), 3) == 0) % 1, 4, 7, etc.
                    ref = text(i*dim_w-dim_w/2, j*dim_h-dim_h/2, strcat(L1((j+2)/3), L1((i+2)/3)));
                    set(ref, 'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle');
                    set(ref, 'Visible', 'on');
                    set(ref, 'FontSize', 7);
                end
            end
        end
    end

    %% Wait for Bluetooth request
    
    errorText = text(max_x/2, max_y/2, 'Waiting for request...');
    set(errorText, 'FontSize', 18, 'Color', 'k', 'BackgroundColor', 'y');
    set(errorText, 'HorizontalAlignment', 'center', 'VerticalAlignment', 'top');
    set(errorText, 'Visible', 'on');   
    
    % Bluetooth Read
    [heightMarker, readError] = NXTRead(nxt);
    heightMarker = double(heightMarker);

    % exit if GUI is closed
    if ~ishandle(hObject)
        fclose(nxt);
        break;
    end
    
    if readError == 1
        pause(0.2) % give program some rest
        continue;
    else     
        startVision = tic; % start timer
        % Turn message off
        delete(errorText);
    end
    
    % To avoid an error with the first iteration
    try
        elapsedTime = toc(start_t); % Time since last successful request
    catch
        elapsedTime = 6;
    end
    requestTime = tic;

    % This loop verifies that the students have waited 5 seconds
    if (elapsedTime < 5.00)
        Error = 32; % System is busy
        xCoor = 0;
        yCoor = 0;
        NXTWrite(nxt, xCoor, yCoor, Error);
        continue;
    end
    
    %% Determine Position

    % Check for the radio button pressed
    value = modeSelection(hObject, eventdata);

    % Manual will require the user to select the error message, then select
    % the location. This information will be sent to the NXT.
    if (value == 2) % for manual
        %% User click

        % reset radio button selection to "manual override engaged"
        set(handles.uipanel1, 'SelectedObject', handles.radiobutton2);         

        % Displaying a message asking for the user to click 
        h = text(max_x/2, 1.05*max_y, 'Click on a square to indicate robot location');
        set(h, 'FontSize', 14, 'Color', 'k');
        set(h, 'HorizontalAlignment', 'center', 'VerticalAlignment', 'top');

        [success, xCoor, yCoor] = userClick(h, dim_w, dim_h, horizontal, ...
            vertical, obstacle);

        % ginput failed because the GUI was closed
        if success == 0
            fclose(nxt);
            break;
        end

        selection = errorSelection(hObject, eventdata, handles);

        % Simulates an error code being sent (according to communication
        % specification) Set coordinates to 0 if error > 2
        Error = 2^(selection-1);
        if (finalVersion && (Error ~= 2))
            Error = Error + 2;
        end

    % Automatic will use the camera to determine the ALV location. Erik's
    % function will return the coordinates and the error. If there is any
    % error, the coordinates should be set to (0, 0).
    elseif (value == 1) % for automatic

        % Call the vision system
        [xCoor, yCoor, Error] = visionTracker.getPoints(heightMarker, ...
            rotMat, transVec);
        
        % Plot a blue x in the exact location
        plot(xCoor, yCoor, 'bx', 'MarkerSize', 30, 'LineWidth', 3);

        h = text(max_x/2, 1.05*max_y, 'The ALV location is shown below');
        set(h, 'FontSize', 16, 'Color', 'k', 'BackgroundColor', 'y');
        set(h, 'HorizontalAlignment', 'center', 'VerticalAlignment', 'top');
        
        % Round the result
        xCoor = cast(round(xCoor), 'int16');
        yCoor = cast(round(yCoor), 'int16');
        Error = cast(Error, 'int16');

    % Semiautomatic will use Erik's code to display the automatic
    % interpreation of the ALV location. The TA must then confirm the error
    % message is appropriate and confirm the location.
    elseif (value == 3) % for semiautomatic

        % Call the vision system
        [xCoor, yCoor, Error] = visionTracker.getPoints(heightMarker, ...
            rotMat, transVec);
        
        % preset errors
        if (Error == 1) % no error
            set(handles.uipanel1, 'SelectedObject', handles.radiobutton2); 
        elseif (Error == 4) % out of bounds
            set(handles.uipanel1, 'SelectedObject', handles.radiobutton3); 
        else % not seen
            set(handles.uipanel1, 'SelectedObject', handles.radiobutton4); 
        end
        
        % Plot a blue x in the exact locationlt
        plot(xCoor, yCoor, 'bx', 'MarkerSize', 30, 'LineWidth', 3);

        % Displaying a message asking for the user to click
        h = text(max_x/2, 1.05*max_y, 'Click on a square to confirm location');
        set(h, 'FontSize', 16, 'Color', 'k', 'BackgroundColor', 'y');
        set(h, 'HorizontalAlignment', 'center', 'VerticalAlignment', 'top');

        [success, xCoor, yCoor] = userClick(h, dim_w, dim_h, horizontal,...
            vertical, obstacle);

        % ginput failed because the GUI was closed
        if success == 0
            fclose(nxt);
            break;
        end

        selection = errorSelection(hObject, eventdata, handles);

        % Simulates an error code being sent (according to communication
        % specification) Set coordinates to 0 if error > 2
        Error = 2^(selection-1);
        if (Error ~= 2)
            Error = Error + 2;
        end
    end
           
    %% Return data to NXT
    % Simulates an error code being sent (according to communication
    % specification) Set coordinates to 0 if error > 4 (>6 if +2 for manual
    % override
    if Error > 6
        xCoor = 0;
        yCoor = 0;
    else
        start_t = requestTime; % Start the timer after a successful return
    end

    plot(xCoor, yCoor, 'rx');
    NXTWrite(nxt, xCoor, yCoor, Error);        
       
    hold off;
    
    if (value == 1) % automatic - record things
       time = toc(startVision);      
       fprintf('NXT response took %f sec\n', time);
       visionTracker.saveImageIntern2(rotMat, transVec, ...
            time, Error, double(xCoor), double(yCoor));       
       fprintf('Complete cycle took %f sec\n', toc(startVision));
    end
    
    pause(1) % give some time to print
end        
end

function LocationTrackerV3_OutputFcn(hObject, eventdata, handles)
% runs when gui closes
end

function LocationTrackerStudentv2_CloseRequestFcn(hObject, eventdata, handles)
% This function checks for a user request to close the GUI
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
    delete(gcf);

    data = get(handles.axes1);
    data.stop = true;

    return;
end

function selection = errorSelection(hObject, eventdata, handles)
% This function evaluates the error selection within the button group for
% use in the main script.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

    handles = guidata(hObject);

    switch get(get(handles.uipanel1,'SelectedObject'),'Tag')
        case 'radiobutton1' % for "No error"
            selection = 1;
        case 'radiobutton2' % for "Error: Manual override engaged"
            selection = 2;
        case 'radiobutton3' % for "Error: Out of bounds detected"
            selection = 3;
        case 'radiobutton4' % for "Error: ALV marker not seen"
            selection = 4;
        case 'radiobutton5' % for "Error: LSTS system error"
            selection = 5;
        case 'radiobutton6' % for "Error: server busy. Try again later"
            selection = 6;
    end
end

function value = modeSelection(hObject, eventdata)
% This function evaluates the button selection within the button group for
% use in the main script.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

    handles = guidata(hObject);

    switch get(get(handles.uipanel2, 'SelectedObject'),'Tag')
        case 'radioAuto' % for automatic
            value = 1;
        case 'radioManual' % for manual
            value = 2;
        case 'radioSemi' % for semiautomatic
            value = 3;
    end
end

%% User Click Stuff
function [success, xCoor, yCoor] = userClick(h, dim_w, dim_h, horizontal,...
    vertical, obstacle)

clicked = false;

% If the figure has been closed, the error will be caught and the
% program will terminate
try
    while clicked == false
        [x, y, button] = ginput(1);
        if button == 1 % The left mouse was clicked
            if (x > 1.05*dim_w*horizontal) 
                % The user clicked in a radio button
                % This won't register as a location click
                continue;
            else
                % Finding which box the click was in
                x_box = floor(x/dim_w + 1);
                y_box = floor(y/dim_h + 1);

                % Ensure this click isn't in an obstacle location
                yCheck = 1 + vertical - y_box; % Because the figure coordinates
                % are flipped from the array coordinates
                
                % So it doesn't try to access OOB index
                if (yCheck >= 1) && (yCheck <= vertical) && (x_box >= 1) ...
                        && (x_box <= horizontal) 
                    if obstacle(yCheck, x_box) == 1
                        % This indicates an obstacle is here, click won't
                        % register
                        disp('Obstacle is here');
                        continue;
                    else
                        clicked = true;
                    end
                else    
                    clicked = false;
                end
            end
        end    
    end

    % Turning the message off
    set(h, 'Visible', 'off');
    
    % Returns the location of the middle of the box
    xCoor = x_box * dim_w - dim_w / 2;
    yCoor = y_box * dim_h - dim_h / 2;

    % Fill in the box by filling between horizontal lines stretching from the
    % left side of the box to the right
    line_x = [(x_box - 1) * dim_w, (x_box) * dim_w];
    line_y = [y_box * dim_h, y_box * dim_h];
    y_min = (y_box - 1) * dim_h;

    fill = area(line_x, line_y, y_min);
    set(fill, 'FaceColor', [0.5, 0, 0]);
    
    success = 1; % ginput was successful
catch
    xCoor = 0;
    yCoor = 0;
    success = 0; % ginput was unsuccessful
end
end

%% Bluetooth Connection Stuff
function [ nxt ] = NXTInit( NXTName )
%NXTINIT Initializes a Lego NXT Bluetooth connection
%   This function prints to stdout
%   IN:
%       NXTName - string containing the friendly name of the NXT device
%           found at the top of the NXT's screen
%   OUT:
%       nxt - a Bluetooth object containing connection info to the NXT
%           Note that this Bluetooth object has had fopen called on it
%           and is ready for reading/writing

% Initialize nxt object as empty
nxt = [];

% Check existing BT connections for the device
for device = instrfind()
    if strfind(device.name, NXTName)
        if strcmp(device.status, 'open')
            % Restart device channel and use it
            fclose(device);
            fopen(device);
            nxt = device;
            break;
        end
        % Delete the device listing if closed
        delete(device);
    end
end

% Device not found in active listings, time to scan for it
if isempty(nxt)
    disp('Device not found in active connections.');
    % Run Bluetooth scan for all visible devices
    fprintf(1, '\nBeginning local Bluetooth scan...\n');
    scan = instrhwinfo('Bluetooth');
    
    % Check if the given name was found
    if ismember(NXTName, scan.RemoteNames)
        disp('Found device, preparing connection...')
    else
        msgID = 'NXTInit:notFound';
        msg = 'Device not found, ensure it is visible';
        throw(MException(msgID, msg));
    end

    % Display TA instructions for pairing
    fprintf(1, '\n');
    disp('If this is the first pairing:');
    disp('    1. Watch NXT screen for passkey entry menu');
    disp('    2. Select the default passkey (1234)');
    disp('    3. On Windows, right-click the Bluetooth icon in the task bar');
    disp('       and select "Allow a Device to Connect"');
    disp('    4. Then enter the passkey (1234)');
    disp('Otherwise just wait for connection to establish');
    nxt = Bluetooth(NXTName, 1);
else
    disp('Device found in active connections, rejoining');
end

% Open the port to the device if it wasn't found in the active list
if strcmp(nxt.Status, 'closed')
    try 
        fopen(nxt);
    catch err
        if strcmp(err.identifier, 'instrument:fopen:opfailed')
            msgID = ['NXTInit:', err.identifier];
            msg = 'Failed to open port to NXT!';
            throw(MException(msgID, msg));
        else
            rethrow(err)
        end
    end
end
fprintf(1, '\nConnection succesful!\n');
% Force print to screen before moving on
drawnow('update');

end

function [ ] = NXTWrite( nxt, xCoord, yCoord, error )
%NXTWrite Write dataOut to NXT connected over Bluetooth
%   IN:
%       nxt - Bluetooth object with opened stream in main program
%       xCoord - signed 16-bit integer of NXT's X coordinate in mm
%       yCoord - signed 16-bit integer of NXT's Y coordinate in mm
%       error - unsigned 16-bit integer of error codes being sent out
%   OUT:
%       None

% Sending NXT direct messaging commands, packet structure as follows from
% LEGO Mindstorm NXT'reme Direct Commands documentation
%
% Byte 0:       Least Significant Byte of command length
% Byte 1:       Most Significant Byte of command length
% Byte 2:       Command type - 0x80 = direct command telegram, no response
%                                     required from NXT
% Byte 3:       Command - 0x09 = MESSAGEWRITE
% Byte 4:       Inbox Number - 0x00 = default mailbox
% Byte 5:       Message size - 0x06 = transmission of 3 16-bit words for
%                                     use with getMessageWithParm[]
%                                     (3 words * 2 bytes = 6 bytes total)
% ---------------- Following bytes are ENGR 14X defined -------------------
% Byte 6-7:     LSB and MSB, respectively, of error being transmitted
% Byte 8-9:     LSB and MSB, respectively, of xCoord being transmitted
% Byte 10-11:   LSB and MSB, respectively, of yCoord codes being transmitted

msgLen = uint8(6);                  % 3 words * 2 bytes per word is 6 bytes
cmdLen = uint16(msgLen + 4);        % 4 bytes for command metadata
cmdType = uint8(128);               % 0x80 is direct command, no response
cmd = uint8(9);                     % 0x09 is a MESSAGEWRITE instruction
inbox = uint8(0);                   % write to inbox 0 by default

fprintf('%s - write e = %d, x = %d, y = %d\n',...
    datestr(now()), error, xCoord, yCoord);

xCoord = int16(xCoord);
yCoord = int16(yCoord);
error = uint16(error);

% Construct outgoing packet as a byte array
% See spec at top of file
rawOut = [bitand(cmdLen, 255),...   Bit mask off the MSB of the command length
          bitshift(cmdLen, -8),...     Logical shift the MSB down to the LSB
          cmdType,...               
          cmd,...
          inbox,...
          msgLen,...
          swapbytes(typecast(uint16(error), 'uint8')), ... % swap LSB and MSB
          swapbytes(typecast(int16(xCoord), 'uint8')), ... % swap LSB and MSB
          swapbytes(typecast(int16(yCoord), 'uint8'))]; % swap LSB and MSB          

% Check if BT communication interrupted for some reason
if strcmp(nxt.status, 'closed')
    try
        % Reopen if we can
        fopen(nxt);
        warning('BT channel was closed, reopened successfully');
    catch err
        % Catch an fopen failure and throw exception to upper level callers
        if strcmp(err.identifier, 'instrument:fopen:opfailed')
            msgID = ['NXTWrite:', err.identifier];
            msg = ['BT channel closed and cannot be opened on this end,'...
                ' please disconnect from NXT menu.'];
            throw(MException(msgID, msg));
        else
            rethrow(err);
        end
    end
end

% Write out data
try
    fwrite(nxt, rawOut, 'uint8');
catch err
    if strcmp(err.identifier, 'instrument:fwrite:opfailed')
        try
            fclose(nxt);
            fopen(nxt);
            fwrite(nxt, rawOut, 'uint8');
        catch err2
            msgID = ['NXTWrite:', err.identifier];
            msg = ['Computer BT port registers as open but NXT ',...
                'connection could not be written to'];
            throw(MException(msgID, msg));
        end
    else
        rethrow(err);
    end
end

end

function [ data, error ] = NXTRead( nxt )
%NXTRead 1 second blocking read from NXT Bluetooth channel
%   IN:
%       nxt - Bluetooth object with open connection to NXT device
%   OUT:
%       data - height as 16-bit word of data from the NXT
%       error - flag set to 1 if no request received in the 1 second
%           alloted blocking read phase, 0 if data received

sync = false;    
time = tic;

% Initialize outputs to error values
data = 0;
error = 1;

while (sync == false)
    pause(0.05);
    % Checks to make sure that the system has not timed out
    if (toc(time) > 1.0)
        % Return variables initialied to error values, just break and
        % return
        break
    end
      
    % A full request is 10 bytes long, await receipt of at least that many
    % bytes
    if (nxt.BytesAvailable < 10)
        continue
    end
    
    % Attempt to read metadata series: 0x0A008009
    % 0x0A - LSB packet size = 10
    % 0x00 - MSB packet size = 0
    % 0x80 - direct command message, no reply requested
    % 0x09 - MESSAGEWRITE command
    if (fread(nxt, 1) == 10)
        if (fread(nxt, 1) == 0)
            if (fread(nxt, 1) == 128)
                if (fread(nxt, 1) == 9)
                    sync = true;
                else
                    sync = false;
                end
            else
                sync = false;
            end
        else
            sync = false;
        end
    else
        sync = false;
    end

    % Continue to wait if sync bytes not received
    if (~sync)
        disp('Incorrect BT message.')
%             % Stop waiting for data if the figure is closed
%             if (~ishandle(LocationTrackerV3))
%                 break;
%             end
        continue
    else
        % Packet received successfully
        error = 0;
        % Get packet metadata
        mailbox = fread(nxt, 1);
        numBytes = fread(nxt, 1);
        raw = fread(nxt, 2);
        
        % Parse the 16 bit words from the last 2 bytes
        % Data is sent little endian, LSB first
        % 
        % Create 16-bit integers from each byte
        % Shift the secondary byte (MSB) to the left by 8
        % Bit-wise OR both number together to merge into a single value
        % get signed integer
        data = bitor(bitshift(int16(raw(2)), 8), int16(raw(1)));
        fprintf('%s - read  h = %d\n', datestr(now()), data);
        flushinput(nxt);
        % fread(nxt, 4); % flush rest of buffer - testing
    end
end
end
