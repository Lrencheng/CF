%% 互补滤波实现姿态估计与位置跟踪
% 使用加速度计、陀螺仪、磁力计和GPS数据，采用互补滤波算法进行姿态估计和位置跟踪。
% 支持可视化误差和三维轨迹。

% 加载数据
ld = load('CircularTrajectorySensorData.mat');
Fs = ld.Fs;
gpsFs = ld.gpsFs;
ratio = Fs./gpsFs;
refloc = ld.refloc;
%ground truth
trajOrient = ld.trajData.Orientation;
trajVel = ld.trajData.Velocity;
trajPos = ld.trajData.Position;
trajAcc = ld.trajData.Acceleration;
trajAngVel = ld.trajData.AngularVelocity;

%imu测量数据,写到sensor数据帧中
sensordata=struct(...
                'imu',struct(...
                    'accel',[0,0,0],...
                    'gyro',[0,0,0],...
                    'mag',[0,0,0] ...
                ),...
                'gps',struct(...
                    'lla',[0,0,0],...
                    'gpsvel',[0,0,0]...
                )...
            );
sensordata.imu.accel=ld.accel;
sensordata.imu.gyro=ld.gyro;
sensordata.imu.mag=ld.mag;
sensordata.gps.lla=ld.lla;
sensordata.gps.gpsvel=ld.gpsvel;
% 互补滤波参数
kp_accel=0.05; % 加速度计权重
kp_mag=0.1; % 磁力计权重
kp_gps = 0.98;%gps权重
dt = 1/Fs;

% 初始化
N = size(accel,1);
estOrient = quaternion.ones(N,1); % 四元数姿态
estPos = zeros(N,3);              % 位置
estVel = zeros(N,3);              % 速度
gyrobias=zeros(N,3);

Nav = 100;
initstate = zeros(28,1);
initstate(1:4) = compact( meanrot(trajOrient(1:Nav))); 
initstate(5:7) = mean( trajAngVel(10:Nav,:), 1);
initstate(8:10) = mean( trajPos(1:Nav,:), 1);
initstate(11:13) = mean( trajVel(1:Nav,:), 1);
initstate(14:16) = mean( trajAcc(1:Nav,:), 1);
initstate(23:25) = ld.magField;
initstate(20:22) = deg2rad([3.125 3.125 3.125]);


initOrient= quaternion(initstate(1),initstate(2),initstate(3),initstate(4)); % 初始姿态
initPos = initstate(8:10); % 初始位置
initVel = initstate(11:13); % 初始速度
initAcc= initstate(14:16); % 初始加速度
initmag = initstate(23:25)'; % 初始磁场向量
% 初始姿态（用加速度计和磁力计估算）
%estOrient(1) = estimateInitialOrientation(accel(1,:), mag(1,:));

estOrient(1)= quatnormalize(initOrient); % 使用初始化的四元数
estPos(1,:) = initPos; % 初始位置
estVel(1,:) = initVel; % 初始速度
% 可视化
useErrScope = true; % 打开流误差图。
usePoseView = true; % 打开3D姿态查看器。
if usePoseView
    posescope = PoseViewerWithSwitches(...
        'XPositionLimits', [-30 30], ...
        'YPositionLimits', [-30, 30], ...
        'ZPositionLimits', [-10 10]);
end
f = gcf;

if useErrScope
    errscope = HelperScrollingPlotter(...
        'NumInputs', 4, ...
        'TimeSpan', 10, ...
        'SampleRate', Fs, ...
        'YLabel', {'degrees', ...
        'meters', ...
        'meters', ...
        'meters'}, ...
        'Title', {'四元数距离', ...
        '位置X误差', ...
        '位置Y误差', ...
        '位置Z误差'}, ...
        'YLimits', ...
        [ -1, 30
        -2, 2
        -2 2
        -2 2]);
end

gpsPos = trajPos(1,:); % 初始GPS位置
% 互补滤波主循环
for ii = 2:N
    %陀螺仪积分
    q_prev = estOrient(ii-1);
    omega = gyro(ii,:);
    omega_quat= quaternion(0, omega(1), omega(2), omega(3)); % 转换为四元数
    q_dot = 0.5 * quatmultiply(q_prev, omega_quat);
    q_gyro = q_prev +q_dot*dt;
    q_gyro = quatnormalize(q_gyro);

    q_acc=q_gyro;
    if (f.UserData.Accelerometer) 
        [q_acc,error_acc]=acc_correct(q_gyro,accel(ii,:));
    end
    q_mag=q_acc;
    if (f.UserData.Magnetometer) 
        [q_mag,error_mag]=mag_correct(q_acc,mag(ii,:),initmag);
    end
    %融合滤波
    temp=slerp(q_gyro,q_acc,kp_accel);
    estOrient(ii)=quatnormalize(slerp(temp,q_mag,kp_mag));
    %误差更新
    %gyrobias(ii,:)=gyrobias(ii-1,:)+kp_accel*error_acc+kp_mag*error_mag;
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %获得运动加速度
    grav_acc = rotatepoint(conj(estOrient(ii)), [0, 0, 9.8]);
    acc_body = accel(ii,:) - grav_acc; % 减去重力加速度
    acc_nav=rotatepoint(estOrient(ii), acc_body);

    % 速度积分
    estVel(ii,:) = estVel(ii-1,:) + acc_nav * dt; % 使用加速度更新速度
    estPos(ii,:) = estPos(ii-1,:) + estVel(ii,:) * dt;
    
    % GPS校正（简单采样保持）
    if mod(ii, fix(Fs/gpsFs)) == 0
        gpsPos = trajPos(ii,:); % 用真值校正，实际应用可用GPS
    end
    estPos(ii,:) = (1-kp_gps)* estPos(ii,:) + kp_gps* gpsPos;
    % 可视化
    posescope(estPos(ii,:), estOrient(ii), trajPos(ii,:), trajOrient(ii));
    orientErr = rad2deg(dist(estOrient(ii), trajOrient(ii)));
    posErr = estPos(ii,:) - trajPos(ii,:);
    errscope(orientErr, posErr(1), posErr(2), posErr(3));
end

%加速度校正函数
function [q_acc,error]=acc_correct(q_pred,acc)
    if norm(acc)>10.6 || norm(acc)<9.6
        error=zeros(1,3);
        q_acc=q_pred;
    else
        grav_acc=[0,0,1];
        acc=acc./norm(acc); % 归一化加速度
        acc_body=quatrotate(q_pred, grav_acc);
        error=cross(acc,acc_body);
        % 校正四元数
        corr_quat =quaternion(1, error(1), error(2), error(3));
        q_acc = quatmultiply(q_pred, corr_quat);
        q_acc = q_acc./norm(q_acc);
    end
end

%磁力计校正函数
function [q_mag,error]=mag_correct(q_pred,mag,init)
    mag_base=init;
    mag_base(3)=0;
    mag_base=mag_base./norm(mag_base);
    mag_body=quatrotate(q_pred,mag_base);
    mag_body(3)=0;%忽略z方向的向量
    mag_body=mag_body./norm(mag_body);

    mag(3)=0;
    mag=mag./norm(mag);

    error=cross(mag,mag_body);
    error = [0, 0, error(3)];%保留偏航方向
    %校正四元数
    corr_quat = quaternion(1, error(1), error(2), error(3));
    corr_quat=quatnormalize(corr_quat);
    q_mag=quatmultiply(q_pred,corr_quat);
    q_mag=q_mag./norm(q_mag);
end
%{
function [q_corr, error] = mag_correct(q, mag_norm, Kp_mag)
% 使用磁力计校正偏航角
m_ref = [1, 0, 0]; % 地磁参考向量 (北方向)

% 当前姿态下的地磁向量
m_body = quatrotate(q, m_ref);

% 忽略Z分量 (仅水平面校正)
m_body(3) = 0; 
m_body = m_body / norm(m_body);

% 计算误差向量 (传感器坐标系)
error = cross(mag_norm, m_body);

% 仅保留偏航方向分量
error = [0, 0, error(3)]; 

% 校正四元数
corr_quat =quaternion(1, Kp_mag*error(1), Kp_mag*error(2), Kp_mag*error(3));
q_corr = quatmultiply(q, corr_quat);
q_corr = q_corr ./ norm(q_corr);
end

function v_rot = quatRotate(q_in, v)
% 使用四元数旋转向量
q=compact(q_in);
q0 = q(1); q1 = q(2); q2 = q(3); q3 = q(4);
vx = v(1); vy = v(2); vz = v(3);

% 旋转矩阵计算
v_rot = zeros(1,3);
v_rot(1) = (1 - 2*q2^2 - 2*q3^2)*vx + (2*q1*q2 - 2*q0*q3)*vy + (2*q1*q3 + 2*q0*q2)*vz;
v_rot(2) = (2*q1*q2 + 2*q0*q3)*vx + (1 - 2*q1^2 - 2*q3^2)*vy + (2*q2*q3 - 2*q0*q1)*vz;
v_rot(3) = (2*q1*q3 - 2*q0*q2)*vx + (2*q2*q3 + 2*q0*q1)*vy + (1 - 2*q1^2 - 2*q2^2)*vz;
end
%}