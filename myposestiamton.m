%% �����˲�ʵ����̬������λ�ø���
% ʹ�ü��ٶȼơ������ǡ������ƺ�GPS���ݣ����û����˲��㷨������̬���ƺ�λ�ø��١�
% ֧�ֿ��ӻ�������ά�켣��

% ��������
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

%imu��������,д��sensor����֡��
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
% �����˲�����
kp_accel=0.05; % ���ٶȼ�Ȩ��
kp_mag=0.1; % ������Ȩ��
kp_gps = 0.98;%gpsȨ��
dt = 1/Fs;

% ��ʼ��
N = size(accel,1);
estOrient = quaternion.ones(N,1); % ��Ԫ����̬
estPos = zeros(N,3);              % λ��
estVel = zeros(N,3);              % �ٶ�
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


initOrient= quaternion(initstate(1),initstate(2),initstate(3),initstate(4)); % ��ʼ��̬
initPos = initstate(8:10); % ��ʼλ��
initVel = initstate(11:13); % ��ʼ�ٶ�
initAcc= initstate(14:16); % ��ʼ���ٶ�
initmag = initstate(23:25)'; % ��ʼ�ų�����
% ��ʼ��̬���ü��ٶȼƺʹ����ƹ��㣩
%estOrient(1) = estimateInitialOrientation(accel(1,:), mag(1,:));

estOrient(1)= quatnormalize(initOrient); % ʹ�ó�ʼ������Ԫ��
estPos(1,:) = initPos; % ��ʼλ��
estVel(1,:) = initVel; % ��ʼ�ٶ�
% ���ӻ�
useErrScope = true; % �������ͼ��
usePoseView = true; % ��3D��̬�鿴����
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
        'Title', {'��Ԫ������', ...
        'λ��X���', ...
        'λ��Y���', ...
        'λ��Z���'}, ...
        'YLimits', ...
        [ -1, 30
        -2, 2
        -2 2
        -2 2]);
end

gpsPos = trajPos(1,:); % ��ʼGPSλ��
% �����˲���ѭ��
for ii = 2:N
    %�����ǻ���
    q_prev = estOrient(ii-1);
    omega = gyro(ii,:);
    omega_quat= quaternion(0, omega(1), omega(2), omega(3)); % ת��Ϊ��Ԫ��
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
    %�ں��˲�
    temp=slerp(q_gyro,q_acc,kp_accel);
    estOrient(ii)=quatnormalize(slerp(temp,q_mag,kp_mag));
    %������
    %gyrobias(ii,:)=gyrobias(ii-1,:)+kp_accel*error_acc+kp_mag*error_mag;
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %����˶����ٶ�
    grav_acc = rotatepoint(conj(estOrient(ii)), [0, 0, 9.8]);
    acc_body = accel(ii,:) - grav_acc; % ��ȥ�������ٶ�
    acc_nav=rotatepoint(estOrient(ii), acc_body);

    % �ٶȻ���
    estVel(ii,:) = estVel(ii-1,:) + acc_nav * dt; % ʹ�ü��ٶȸ����ٶ�
    estPos(ii,:) = estPos(ii-1,:) + estVel(ii,:) * dt;
    
    % GPSУ�����򵥲������֣�
    if mod(ii, fix(Fs/gpsFs)) == 0
        gpsPos = trajPos(ii,:); % ����ֵУ����ʵ��Ӧ�ÿ���GPS
    end
    estPos(ii,:) = (1-kp_gps)* estPos(ii,:) + kp_gps* gpsPos;
    % ���ӻ�
    posescope(estPos(ii,:), estOrient(ii), trajPos(ii,:), trajOrient(ii));
    orientErr = rad2deg(dist(estOrient(ii), trajOrient(ii)));
    posErr = estPos(ii,:) - trajPos(ii,:);
    errscope(orientErr, posErr(1), posErr(2), posErr(3));
end

%���ٶ�У������
function [q_acc,error]=acc_correct(q_pred,acc)
    if norm(acc)>10.6 || norm(acc)<9.6
        error=zeros(1,3);
        q_acc=q_pred;
    else
        grav_acc=[0,0,1];
        acc=acc./norm(acc); % ��һ�����ٶ�
        acc_body=quatrotate(q_pred, grav_acc);
        error=cross(acc,acc_body);
        % У����Ԫ��
        corr_quat =quaternion(1, error(1), error(2), error(3));
        q_acc = quatmultiply(q_pred, corr_quat);
        q_acc = q_acc./norm(q_acc);
    end
end

%������У������
function [q_mag,error]=mag_correct(q_pred,mag,init)
    mag_base=init;
    mag_base(3)=0;
    mag_base=mag_base./norm(mag_base);
    mag_body=quatrotate(q_pred,mag_base);
    mag_body(3)=0;%����z���������
    mag_body=mag_body./norm(mag_body);

    mag(3)=0;
    mag=mag./norm(mag);

    error=cross(mag,mag_body);
    error = [0, 0, error(3)];%����ƫ������
    %У����Ԫ��
    corr_quat = quaternion(1, error(1), error(2), error(3));
    corr_quat=quatnormalize(corr_quat);
    q_mag=quatmultiply(q_pred,corr_quat);
    q_mag=q_mag./norm(q_mag);
end
%{
function [q_corr, error] = mag_correct(q, mag_norm, Kp_mag)
% ʹ�ô�����У��ƫ����
m_ref = [1, 0, 0]; % �شŲο����� (������)

% ��ǰ��̬�µĵش�����
m_body = quatrotate(q, m_ref);

% ����Z���� (��ˮƽ��У��)
m_body(3) = 0; 
m_body = m_body / norm(m_body);

% ����������� (����������ϵ)
error = cross(mag_norm, m_body);

% ������ƫ���������
error = [0, 0, error(3)]; 

% У����Ԫ��
corr_quat =quaternion(1, Kp_mag*error(1), Kp_mag*error(2), Kp_mag*error(3));
q_corr = quatmultiply(q, corr_quat);
q_corr = q_corr ./ norm(q_corr);
end

function v_rot = quatRotate(q_in, v)
% ʹ����Ԫ����ת����
q=compact(q_in);
q0 = q(1); q1 = q(2); q2 = q(3); q3 = q(4);
vx = v(1); vy = v(2); vz = v(3);

% ��ת�������
v_rot = zeros(1,3);
v_rot(1) = (1 - 2*q2^2 - 2*q3^2)*vx + (2*q1*q2 - 2*q0*q3)*vy + (2*q1*q3 + 2*q0*q2)*vz;
v_rot(2) = (2*q1*q2 + 2*q0*q3)*vx + (1 - 2*q1^2 - 2*q3^2)*vy + (2*q2*q3 - 2*q0*q1)*vz;
v_rot(3) = (2*q1*q3 - 2*q0*q2)*vx + (2*q2*q3 + 2*q0*q1)*vy + (1 - 2*q1^2 - 2*q2^2)*vz;
end
%}