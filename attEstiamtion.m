%% 使用CompFilter进行姿态估计
% 本示例展示如何使用CompFilter类处理IMU传感器数据并估计设备姿态。
% 使用加速度计、磁力计和陀螺仪测量值来计算设备在空间中的方向。


%   版权所有 2025 The MathWorks, Inc.

%% 加载数据
% 加载预录的传感器数据。轨迹数据通过|waypointTrajectory|生成，
% 圆形轨迹的真实值通过|imuSensor|生成。
% 此处使用的|CircularTrajectorySensorData.mat|文件通过
% |generateCircularTrajSensorData|函数生成。

ld = load('CircularTrajectorySensorData.mat');

Fs = ld.Fs; % IMU采样率
gpsFs = ld.gpsFs; % GPS采样率
refloc = ld.refloc; 

% 真实轨迹数据
trajOrient = ld.trajData.Orientation;
trajAngVel = ld.trajData.AngularVelocity;

%创建数据帧
sensordata = struct(...
        'imu', struct(...
            'accel', zeros(1,3), ...
            'gyro', zeros(1,3), ...
            'mag', zeros(1,3) ...
        ),...
        'gps',struct(...
            'lla',zeros(1,3),...
            'gpsvel',zeros(1,3)...
        )...
);
% 传感器数据
accel = ld.accel;
gyro = ld.gyro;
mag = ld.mag;
lla=ld.lla;
gpsvel=ld.gpsvel;

%% 初始化CompFilter
% 创建一个CompFilter对象来处理IMU测量值

compFilt = CompFilter();

% 滤波器参数配置
config = struct(...
    'filtercoef', struct(...   % 滤波系数
        'kp_accel', 0.1, ...  
        'ki_accel', 0.002, ...    
        'kp_mag', 0.015, ...
        'kp_gps', 0.8 ...
    ), ...
    'fs', struct(...           % 采样频率
        'accelFs', Fs, ...
        'gyroFs', Fs, ...
        'magFs', Fs, ...
        'gpsFs', gpsFs ...  % GPS频率未使用，但需要设置
    ) ...
);

% 初始化传感器
compFilt.SensorInit(refloc, config);

%% 初始化显示工具
% |HelperScrollingPlotter|显示工具支持实时数据的滚动时间序列图
% 可用于可视化姿态估计误差。

useErrScope = true; % 启用误差图

if useErrScope
    errscope = HelperScrollingPlotter(...
        'NumInputs', 1, ...
        'TimeSpan', 10, ...
        'SampleRate', Fs, ...
        'YLabel', 'degrees', ...
        'Title', '四元数误差', ...
        'YLimits', [-1, 30]);
end

record_err=zeros(size(accel,1),1);
calOrient=zeros(size(accel,1),4);
%% 主循环
% 在此循环中，算法处理IMU传感器数据并估计姿态。
%校准
for ii=1:size(accel,1)
    % 准备传感器数据
    sensordata.imu.accel=accel(ii,:);
    sensordata.imu.gyro=gyro(ii,:);
    sensordata.imu.mag=mag(ii,:);
    sensordata.gps.lla=lla(ii,:);
    sensordata.gps.gpsvel=gpsvel(ii,:);
    % 更新传感器数据
    compFilt.getdata(sensordata);
    %初始100个采样周期内进行校准
    if ii<=100
        state=compFilt.imu_calibrate();
        %compFilt.imu.gyro.bias(1)=0.063662563;
        %compFilt.imu.gyro.bias(2)=0.063662563;
    end
    if ii>100
         % 执行IMU姿态更新
        compFilt.ImuUpdate();
    
        % 获取估计的姿态（从导航坐标系到体坐标系）
        estimatedOrient = compFilt.imu.q;
        calOrient(ii,:)=compact(estimatedOrient);
         % 计算姿态误差并显示
        if useErrScope && exist('errscope', 'var')
            orientErr = rad2deg(dist(estimatedOrient,trajOrient(ii)));
            errscope(orientErr);
            record_err(ii)=orientErr;
        end
    end
end


%% 绘制四元数解算误差
% Plot the recorded error data

% Create time vector
timeX = (0:size(accel,1)-1) / Fs;

% Plot recorded error
figure;
plot(timeX, record_err);
title('Attitude Estimation Error');
ylabel('Error (degrees)');
xlabel('Time (s)');

%% 总结
% CompFilter类通过融合IMU传感器数据来估计设备姿态
% 在许多应用中可以替代传统的姿态估计算法。