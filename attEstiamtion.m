%% ʹ��CompFilter������̬����
% ��ʾ��չʾ���ʹ��CompFilter�ദ��IMU���������ݲ������豸��̬��
% ʹ�ü��ٶȼơ������ƺ������ǲ���ֵ�������豸�ڿռ��еķ���


%   ��Ȩ���� 2025 The MathWorks, Inc.

%% ��������
% ����Ԥ¼�Ĵ��������ݡ��켣����ͨ��|waypointTrajectory|���ɣ�
% Բ�ι켣����ʵֵͨ��|imuSensor|���ɡ�
% �˴�ʹ�õ�|CircularTrajectorySensorData.mat|�ļ�ͨ��
% |generateCircularTrajSensorData|�������ɡ�

ld = load('CircularTrajectorySensorData.mat');

Fs = ld.Fs; % IMU������
gpsFs = ld.gpsFs; % GPS������
refloc = ld.refloc; 

% ��ʵ�켣����
trajOrient = ld.trajData.Orientation;
trajAngVel = ld.trajData.AngularVelocity;

%��������֡
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
% ����������
accel = ld.accel;
gyro = ld.gyro;
mag = ld.mag;
lla=ld.lla;
gpsvel=ld.gpsvel;

%% ��ʼ��CompFilter
% ����һ��CompFilter����������IMU����ֵ

compFilt = CompFilter();

% �˲�����������
config = struct(...
    'filtercoef', struct(...   % �˲�ϵ��
        'kp_accel', 0.1, ...  
        'ki_accel', 0.002, ...    
        'kp_mag', 0.015, ...
        'kp_gps', 0.8 ...
    ), ...
    'fs', struct(...           % ����Ƶ��
        'accelFs', Fs, ...
        'gyroFs', Fs, ...
        'magFs', Fs, ...
        'gpsFs', gpsFs ...  % GPSƵ��δʹ�ã�����Ҫ����
    ) ...
);

% ��ʼ��������
compFilt.SensorInit(refloc, config);

%% ��ʼ����ʾ����
% |HelperScrollingPlotter|��ʾ����֧��ʵʱ���ݵĹ���ʱ������ͼ
% �����ڿ��ӻ���̬������

useErrScope = true; % �������ͼ

if useErrScope
    errscope = HelperScrollingPlotter(...
        'NumInputs', 1, ...
        'TimeSpan', 10, ...
        'SampleRate', Fs, ...
        'YLabel', 'degrees', ...
        'Title', '��Ԫ�����', ...
        'YLimits', [-1, 30]);
end

record_err=zeros(size(accel,1),1);
calOrient=zeros(size(accel,1),4);
%% ��ѭ��
% �ڴ�ѭ���У��㷨����IMU���������ݲ�������̬��
%У׼
for ii=1:size(accel,1)
    % ׼������������
    sensordata.imu.accel=accel(ii,:);
    sensordata.imu.gyro=gyro(ii,:);
    sensordata.imu.mag=mag(ii,:);
    sensordata.gps.lla=lla(ii,:);
    sensordata.gps.gpsvel=gpsvel(ii,:);
    % ���´���������
    compFilt.getdata(sensordata);
    %��ʼ100�����������ڽ���У׼
    if ii<=100
        state=compFilt.imu_calibrate();
        %compFilt.imu.gyro.bias(1)=0.063662563;
        %compFilt.imu.gyro.bias(2)=0.063662563;
    end
    if ii>100
         % ִ��IMU��̬����
        compFilt.ImuUpdate();
    
        % ��ȡ���Ƶ���̬���ӵ�������ϵ��������ϵ��
        estimatedOrient = compFilt.imu.q;
        calOrient(ii,:)=compact(estimatedOrient);
         % ������̬����ʾ
        if useErrScope && exist('errscope', 'var')
            orientErr = rad2deg(dist(estimatedOrient,trajOrient(ii)));
            errscope(orientErr);
            record_err(ii)=orientErr;
        end
    end
end


%% ������Ԫ���������
% Plot the recorded error data

% Create time vector
timeX = (0:size(accel,1)-1) / Fs;

% Plot recorded error
figure;
plot(timeX, record_err);
title('Attitude Estimation Error');
ylabel('Error (degrees)');
xlabel('Time (s)');

%% �ܽ�
% CompFilter��ͨ���ں�IMU�����������������豸��̬
% �����Ӧ���п��������ͳ����̬�����㷨��