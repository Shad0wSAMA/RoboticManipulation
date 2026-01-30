% Read the position of the dynamixel horn with the torque off
% The code executes for a given amount of time then terminates


clc;
clear all;

lib_name = '';

if strcmp(computer, 'PCWIN')
  lib_name = 'dxl_x86_c';
elseif strcmp(computer, 'PCWIN64')
  lib_name = 'dxl_x64_c';
elseif strcmp(computer, 'GLNX86')
  lib_name = 'libdxl_x86_c';
elseif strcmp(computer, 'GLNXA64')
  lib_name = 'libdxl_x64_c';
elseif strcmp(computer, 'MACI64')
  lib_name = 'libdxl_mac_c';
end

% Load Libraries
if ~libisloaded(lib_name)
    [notfound, warnings] = loadlibrary(lib_name, 'dynamixel_sdk.h', 'addheader', 'port_handler.h', 'addheader', 'packet_handler.h');
end

%% ---- Control Table Addresses ---- %%
ADDR_MAX_POS = 48;
ADDR_MIN_POS = 52;

ADDR_PRO_TORQUE_ENABLE       = 64;           % Control table address is different in Dynamixel model
ADDR_PRO_GOAL_POSITION       = 116; 
ADDR_PRO_PRESENT_POSITION    = 132; 
ADDR_PRO_OPERATING_MODE      = 11;
ADDR_PRO_PROFILE_VELOCITY    = 112;
 
%% ---- Other Settings ---- %%

% Protocol version
PROTOCOL_VERSION            = 2.0;          % See which protocol version is used in the Dynamixel

% Default setting

DXL_ID1 = 11;
DXL_ID2 = 12;
DXL_ID3 = 13;
DXL_ID4 = 14;
DXL_GRIP = 15;              % 夹爪电机

DXL_IDS = [DXL_ID1 DXL_ID2 DXL_ID3 DXL_ID4 DXL_GRIP];   % 统一管理


BAUDRATE                    = 1000000;
DEVICENAME                  = 'COM19';       % Check which port is being used on your controller
                                            % ex) Windows: 'COM1'   Linux: '/dev/ttyUSB0' Mac: '/dev/tty.usbserial-*'
                                            
TORQUE_ENABLE               = 1;            % Value for enabling the torque
TORQUE_DISABLE              = 0;            % Value for disabling the torque
DXL_MINIMUM_POSITION_VALUE  = -150000;      % Dynamixel will rotate between this value
DXL_MAXIMUM_POSITION_VALUE  = 150000;       % and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
DXL_MOVING_STATUS_THRESHOLD = 20;           % Dynamixel moving status threshold

ESC_CHARACTER               = 'e';          % Key for escaping loop

COMM_SUCCESS                = 0;            % Communication Success result value
COMM_TX_FAIL                = -1001;        % Communication Tx Failed
MAX_POS                     = 3400; 
MIN_POS                     = 600; 
VELOCITY                    = 1000;

%% ------------------ %%

% Initialize PortHandler Structs
% Set the port path
% Get methods and members of PortHandlerLinux or PortHandlerWindows
port_num = portHandler(DEVICENAME);

% Initialize PacketHandler Structs
packetHandler();

index = 1;
dxl_comm_result = COMM_TX_FAIL;           % Communication result
dxl_goal_position = [DXL_MINIMUM_POSITION_VALUE DXL_MAXIMUM_POSITION_VALUE];         % Goal position
target_pos_list = [1000, 3000];

dxl_error = 0;                              % Dynamixel error
dxl_present_position = 0;                   % Present position


% Open port
if (openPort(port_num))
    fprintf('Port Open\n');
else
    unloadlibrary(lib_name);
    fprintf('Failed to open the port\n');
    input('Press any key to terminate...\n');
    return;
end


% Set port baudrate
if (setBaudRate(port_num, BAUDRATE))
    fprintf('Baudrate Set\n');
else
    unloadlibrary(lib_name);
    fprintf('Failed to change the baudrate!\n');
    input('Press any key to terminate...\n');
    return;
end

% Disable Dynamixel Torque
for id = DXL_IDS
    write1ByteTxRx(port_num, PROTOCOL_VERSION, id, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE);
end

% Put actuator into Position Control Mode
for id = DXL_IDS
    write1ByteTxRx(port_num, PROTOCOL_VERSION, id, ADDR_PRO_OPERATING_MODE, 3);
end

% Enable Dynamixel Torque
for id = DXL_IDS
    write1ByteTxRx(port_num, PROTOCOL_VERSION, id, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE);
end

% Set Profile Velocity
%write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID1, ADDR_PRO_PROFILE_VELOCITY, VELOCITY);
%write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID2, ADDR_PRO_PROFILE_VELOCITY, VELOCITY);


dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION);
dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION);

if dxl_comm_result ~= COMM_SUCCESS
    fprintf('%s\n', getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
elseif dxl_error ~= 0
    fprintf('%s\n', getRxPacketError(PROTOCOL_VERSION, dxl_error));
else
    fprintf('Dynamixel has been successfully connected \n');
end



write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID1, ADDR_MAX_POS, MAX_POS); 
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID2, ADDR_MAX_POS, MAX_POS);
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID1, ADDR_MIN_POS, MIN_POS); 
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID2, ADDR_MIN_POS, MIN_POS);
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID1, ADDR_PRO_GOAL_POSITION, 2048);
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID2, ADDR_PRO_GOAL_POSITION, 2048);
pause(1)

i = 0;

while i<360
    goal_pos1 = 2048-sin(i/180*pi)*500;
    goal_pos2 = 2048+sin(i/180*pi)*500;
    write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID1, ADDR_PRO_GOAL_POSITION, goal_pos1);
    write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID2, ADDR_PRO_GOAL_POSITION, goal_pos2);


    dxl1_present_position = read4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID1, ADDR_PRO_PRESENT_POSITION);
    dxl2_present_position = read4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID2, ADDR_PRO_PRESENT_POSITION);
    dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION);
    
    dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION);
    
    if dxl_comm_result ~= COMM_SUCCESS
        fprintf('%s\n', getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
    elseif dxl_error ~= 0
        fprintf('%s\n', getRxPacketError(PROTOCOL_VERSION, dxl_error));
    end

    %fprintf('[ID:%03d] Position: %03d\n', DXL_ID1, typecast(uint32(dxl1_present_position), 'int32'));
    
    T_0 = [1 0 0;
           0 1 0;
           0 0 1];

    theta1 = (dxl1_present_position-2048)/4096*2*pi;

    T_1 = [cos(theta1)  sin(theta1)  80*sin(theta1);
        -sin(theta1)  cos(theta1)  80*cos(theta1);
        0  0   1 ];
    
    theta2 = (dxl2_present_position-2048)/4096*2*pi;

    T_2 = [cos(theta2)  sin(theta2)  60*sin(theta2);
        -sin(theta2)  cos(theta2)  60*cos(theta2);
        0  0   1 ];

    
    T_final = T_2*T_1*T_0
    

    % Set max position limit 

%{
    j = 0;
    while (j<200)
        j = j+1;
        
        % Read present position
        dxl_present_position = read4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_PRO_PRESENT_POSITION);
        dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION);
        
        dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION);
        
        if dxl_comm_result ~= COMM_SUCCESS
            fprintf('%s\n', getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
        elseif dxl_error ~= 0
            fprintf('%s\n', getRxPacketError(PROTOCOL_VERSION, dxl_error));
        end

        fprintf('[ID:%03d] Position: %03d\n', DXL_ID, typecast(uint32(dxl_present_position), 'int32'));

        if ~(abs(dxl_goal_position(index) - typecast(uint32(dxl_present_position), 'int32')) > DXL_MOVING_STATUS_THRESHOLD)
            break;
        end
    end
%}
    pause(0.001);
    i = i+1;
end


% Disable Dynamixel Torque
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID1, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE);
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID2, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE);

dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION);
dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION);
if dxl_comm_result ~= COMM_SUCCESS
    fprintf('%s\n', getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
elseif dxl_error ~= 0
    fprintf('%s\n', getRxPacketError(PROTOCOL_VERSION, dxl_error));
end

% Close port
closePort(port_num);
fprintf('Port Closed \n');

% Unload Library
unloadlibrary(lib_name);

close all;
clear all;
