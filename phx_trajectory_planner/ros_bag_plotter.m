billclear all
close all
clc

bag = rosbag('2017-11-02-18-00-30-Funkflug.bag');

phx_imu = select(bag, 'Topic', '/phx/imu');
motor = select(bag, 'Topic', '/phx/fc/motor_set');
att = select(bag, 'Topic', '/phx/fc/attitude');
alt_lidar = select(bag, 'Topic', '/phx/altitude');
alt_fc = select(bag, 'Topic', '/phx/fc/altitude');
mtr_fc = select(bag, 'Topic', '/phx/fc/motor');
ssh_rc = select(bag, 'Topic', '/phx/ssh_rc');
imu_msgs = readMessages(phx_imu);
mtr_msgs = readMessages(motor);
attitude_msgs = readMessages(att);
alt_lid_msgs = readMessages(alt_lidar);
alt_fc_msgs = readMessages(alt_fc);
mtr_fc_msgs = readMessages(mtr_fc);
ssh = readMessages(ssh_rc);
rc_bag = select(bag, 'Topic', '/phx/fc/rc');
rc = readMessages(rc_bag);
controller = select(bag, 'Topic', '/phx/trajectory_controller');
controller_cmd = readMessages(controller);

thrusts = zeros(length(mtr_msgs), 7);
% first column, thrusts in prozent (Motor [0 1 2 3 4 5]); 7th column time
for i = 1:length(mtr_msgs)
    thrusts_PWM = [mtr_msgs{i,1}.Motor0 mtr_msgs{i,1}.Motor1 mtr_msgs{i,1}.Motor2 mtr_msgs{i,1}.Motor3 mtr_msgs{i,1}.Motor4 mtr_msgs{i,1}.Motor5];
    thrusts(i,1:6) = (thrusts_PWM - 1025).*(100/975);
    thrusts(i,7) = (mtr_msgs{i,1}.Header.Stamp.Nsec - mtr_msgs{1,1}.Header.Stamp.Nsec)*1e-9...
                    + mtr_msgs{i,1}.Header.Stamp.Sec - mtr_msgs{1,1}.Header.Stamp.Sec;
end

thrusts_fc = zeros(length(mtr_fc_msgs), 7);
% first column, thrusts in prozent (Motor [0 1 2 3 4 5]); last column time
for i = 1:length(mtr_fc_msgs)
    thrusts_fc_PWM = [mtr_fc_msgs{i,1}.Motor0 mtr_fc_msgs{i,1}.Motor1 mtr_fc_msgs{i,1}.Motor2 mtr_fc_msgs{i,1}.Motor3 mtr_fc_msgs{i,1}.Motor4 mtr_fc_msgs{i,1}.Motor5];
    thrusts_fc(i,1:6) = (thrusts_fc_PWM - 1025).*(100/975);
    thrusts_fc(i,7) = (mtr_fc_msgs{i,1}.Header.Stamp.Nsec - mtr_fc_msgs{1,1}.Header.Stamp.Nsec)*1e-9...
                    + mtr_fc_msgs{i,1}.Header.Stamp.Sec - mtr_fc_msgs{1,1}.Header.Stamp.Sec;
end

rc_data = zeros(length(rc), 4);
rc_t = zeros(length(rc), 1);
% [aux1, aux2, aux3, aux4]
for i = 1:length(rc)
    rc_t(i) = (rc{i,1}.Header.Stamp.Nsec - rc{1,1}.Header.Stamp.Nsec)*1e-9...
                    + rc{i,1}.Header.Stamp.Sec - rc{1,1}.Header.Stamp.Sec;
    rc_data(i,:) = [rc{i,1}.Aux1, rc{i,1}.Aux2, rc{i,1}.Aux3, rc{i,1}.Aux4];
end
if(length(mtr_msgs) > 10) % regler war aktiv, auf gleiche zeitachse bringen
    rc_t = rc_t - ones(size(rc_t)).*(rc_t(end)-thrusts(end,7));    
end
idx = find(rc_data(:,4)>1600,1,'first');
t_regler_start = rc_t(idx);
while(((rc_data(idx,4))>1600) && (idx < length(rc_t)))
    idx = idx + 1;
end
t_regler_stop = rc_t(idx);

att_time = zeros(length(attitude_msgs),1);
euler = zeros(length(attitude_msgs),3);
for i = 1:length(attitude_msgs)
    t = (attitude_msgs{i,1}.Header.Stamp.Nsec - attitude_msgs{1,1}.Header.Stamp.Nsec)*1e-9...
                    + attitude_msgs{i,1}.Header.Stamp.Sec - attitude_msgs{1,1}.Header.Stamp.Sec;
        att_time(i,1) = t;
        euler(i,:) = [attitude_msgs{i,1}.Yaw, -attitude_msgs{i,1}.Pitch, attitude_msgs{i,1}.Roll]; % ZYX [psi theta phi]
end

imu_time = zeros(length(imu_msgs),1);
acc = zeros(length(imu_msgs),3);
acc_NED = zeros(length(imu_msgs),3);
v = zeros(length(imu_msgs),3);
rates = zeros(length(imu_msgs),3);
euler_imu = zeros(length(imu_msgs),3);
for i = 1:length(imu_msgs)
    t = (imu_msgs{i,1}.Header.Stamp.Nsec - imu_msgs{1,1}.Header.Stamp.Nsec)*1e-9...
                    + imu_msgs{i,1}.Header.Stamp.Sec - imu_msgs{1,1}.Header.Stamp.Sec;
    % select data from same time
    %if((t >= thrusts(1,7)) && (t <= thrusts(length(mtr_msgs),7)))
        imu_time(i,1) = t;
%         q = imu_msgs{i,1}.Orientation;
%         quat = [q.X q.Y q.Z q.W];
%         %imu_data{i,1} = quat2eul(quat,'XYZ');
%         test = quat2eul(quat);
        accel = imu_msgs{i,1}.LinearAcceleration;
        acc(i,:) = [accel.X accel.Y accel.Z];
        phi = euler(i,3);
        theta = euler(i,2);
        acc_NED(i,:) = ([cosd(theta) 0 -sind(theta); ...
                        sind(theta)*sind(phi) cosd(phi) cosd(theta)*sind(phi); ...
                        sind(theta)*cosd(phi) -sind(phi) cosd(theta)*cosd(phi)]'*(acc(i,:)'))';
        if(i==1)
           v(i,:) = imu_time(i,1)*[acc_NED(i,1:2) acc_NED(i,3)-9.80665]; 
        else
           v(i,:) = v(i-1,:) + (imu_time(i,1)-imu_time(i-1,1))*[acc_NED(i,1:2) acc_NED(i,3)-9.80665];  
        end
        
        w = imu_msgs{i,1}.AngularVelocity;
        %rates(i,:) = [1/sqrt(2)*(w.X + w.Y) 1/sqrt(2)*(w.X - w.Y) w.Z];
        rates(i,:) = [w.X -w.Y -w.Z].*2000/8192;
        
        if(i==1)
           euler_imu(i,:) = imu_time(i,1).*rates(i,:)*0.5; 
        else
           euler_imu(i,:) = euler_imu(i-1,:) + (imu_time(i,1)-imu_time(i-1,1)).*rates(i,:); 
        end
        
    %end
end

%k=1;
%alt_lid_time = zeros(length(alt_lid_msgs),1); 
% zeit wird noch nicht
% mitgeschickt!!!!!!!
%alt_lid_time = (linspace(0,imu_time(end,1),length(alt_lid_msgs)))';
alt_lid_time = zeros(length(alt_lid_msgs),1);
h_lid_raw = zeros(length(alt_lid_msgs),1);
h_filter = zeros(length(alt_lid_msgs),1);
%wg = 25;
wg = 4;
for i = 1:length(alt_lid_msgs)
    alt_lid_time(i,1) = (alt_lid_msgs{i,1}.Header.Stamp.Nsec - alt_lid_msgs{1,1}.Header.Stamp.Nsec)*1e-9...
                    + alt_lid_msgs{i,1}.Header.Stamp.Sec - alt_lid_msgs{1,1}.Header.Stamp.Sec;
    % select data from same time
    %if((t >= thrusts(1,7)) && (t <= thrusts(length(mtr_msgs),7)))
    %alt_lid_time(i,1) = t;      
    h_lid_raw(i,1) = alt_lid_msgs{i,1}.EstimatedAltitude;
    if(i ~= 1)
        dt = alt_lid_time(i,1) - alt_lid_time(i-1,1);
        h_filter(i,1) = ( h_lid_raw(i,1)*wg*dt + h_filter(i-1,1) ) / ( 1+wg*dt );
    else
        h_filter(1,1) = h_lid_raw(1,1);
    end
end

alt_fc_time = zeros(length(alt_fc_msgs),1);
h_fc = zeros(length(alt_fc_msgs),2);
for i = 1:length(alt_fc_msgs)
    t = (alt_fc_msgs{i,1}.Header.Stamp.Nsec - alt_fc_msgs{1,1}.Header.Stamp.Nsec)*1e-9...
                    + alt_fc_msgs{i,1}.Header.Stamp.Sec - alt_fc_msgs{1,1}.Header.Stamp.Sec;
    % select data from same time
    %if((t >= thrusts(1,7)) && (t <= thrusts(length(mtr_msgs),7)))
        alt_fc_time(i,1) = t;      
        h_fc(i,1) = alt_fc_msgs{i,1}.EstimatedAltitude;
        h_fc(i,2) = alt_fc_msgs{i,1}.Variation;
       % k = k+1;
    %end
end
% 
% for i = 1:length(rc_msgs)
%     t = (rc_msgs{i,1}.Header.Stamp.Nsec - rc_msgs{1,1}.Header.Stamp.Nsec)*1e-9...
%                     + rc_msgs{i,1}.Header.Stamp.Sec - rc_msgs{1,1}.Header.Stamp.Sec;
%     % select data from same time
%     %if((t >= thrusts(1,7)) && (t <= thrusts(length(mtr_msgs),7)))
%         rc_time(i,1) = t;      
%         rc_roll(i,1) = rc_msgs{i,1}.Roll;
%         rc_pitch(i,1) = rc_msgs{i,1}.Pitch;
%         rc_yaw(i,1) = rc_msgs{i,1}.Yaw;
%         rc_T(i,1) = rc_msgs{i,1}.Throttle;
%        % k = k+1;
%     %end
% end
% 
% for i = 1:length(rc_pilot_msgs)
%     t = (rc_pilot_msgs{i,1}.Header.Stamp.Nsec - rc_pilot_msgs{1,1}.Header.Stamp.Nsec)*1e-9...
%                     + rc_pilot_msgs{i,1}.Header.Stamp.Sec - rc_pilot_msgs{1,1}.Header.Stamp.Sec;
%     % select data from same time
%     %if((t >= thrusts(1,7)) && (t <= thrusts(length(mtr_msgs),7)))
%         rc_pilot_time(i,1) = t;      
%         rc_pilot_roll(i,1) = rc_pilot_msgs{i,1}.Roll;
%         rc_pilot_pitch(i,1) = rc_pilot_msgs{i,1}.Pitch;
%         rc_pilot_yaw(i,1) = rc_pilot_msgs{i,1}.Yaw;
%         rc_pilot_T(i,1) = rc_pilot_msgs{i,1}.Throttle;
%        % k = k+1;
%     %end
% end

cmd = zeros(length(ssh), 4);
% first column, thrusts in prozent (Motor [0 1 2 3 4 5]); 7th column time
for i = 1:length(ssh)
    cmd(i,1) = ssh{i,1}.Roll/2; % roll
    cmd(i,2) = ssh{i,1}.Pitch/2; % pitch
    cmd(i,3) = ssh{i,1}.Aux1; % throttle dT
    cmd(i,4) = (ssh{i,1}.Header.Stamp.Nsec - ssh{1,1}.Header.Stamp.Nsec)*1e-9...
                    + ssh{i,1}.Header.Stamp.Sec - ssh{1,1}.Header.Stamp.Sec; % time
end

%% plot Attitude Dynamics
figure
if(length(mtr_msgs) > 10) % regler war aktiv, auf gleiche zeitachse bringen
    att_time = att_time - ones(size(att_time)).*(att_time(end)-thrusts(end,7));    
end
yyaxis left;
plot(att_time,euler(:,3))
hold on
plot(att_time,euler(:,2))
if(length(mtr_msgs) > 10) % regler war aktiv
    plot(cmd(:,4),cmd(:,1),'s')
    plot(cmd(:,4),cmd(:,2),'x')
end
ylabel('Winkel [??]')
yl = ylim;
ylim([-max(abs(yl)) max(abs(yl))]);
yyaxis right;
if(length(mtr_msgs) > 10) % regler war aktiv, auf gleiche zeitachse bringen
    imu_time = imu_time - ones(size(imu_time)).*(imu_time(end)-thrusts(end,7));    
end
plot(imu_time,rates(:,1))
%hold on
plot(imu_time,rates(:,2))
plot(imu_time,rates(:,3))
ylabel('Drehraten [??/s]')
yl = ylim;
ylim([-max(abs(yl)) max(abs(yl))]);
%plot(att_time,euler(:,1))
plot(t_regler_start.*ones(100,1), (linspace(-max(abs(yl)), max(abs(yl)), 100))','g','LineWidth',2);
plot(t_regler_stop.*ones(100,1), (linspace(-max(abs(yl)), max(abs(yl)), 100))','r','LineWidth',2);
title('Attitude Dynamics')
if(length(mtr_msgs) > 10)
    legend('phi','theta','phi cmd','theta cmd','p','q','r')
else
    legend('phi','theta','p','q','r')
end
xlabel('t [s]')
grid on

%% plot roll / nick regler
figure
subplot(2,1,1)
title('Nickregler')
yyaxis left;
plot(att_time,euler(:,2))
hold on
plot(cmd(:,4),cmd(:,2))
plot(imu_time,rates(:,2))
ylabel('Winkel [??], Drehrate [??/s]')
% yl = ylim;
% ylim([-max(abs(yl)) max(abs(yl))]);
ylim([-100 100])
yyaxis right;
plot(thrusts(:,7),thrusts(:,1))
hold on
plot(thrusts(:,7),thrusts(:,2))
plot(thrusts(:,7),thrusts(:,3))
plot(thrusts(:,7),thrusts(:,4))
plot(t_regler_start.*ones(100,1), (linspace(-max(abs(yl)), max(abs(yl)), 100))','g','LineWidth',2);
plot(t_regler_stop.*ones(100,1), (linspace(-max(abs(yl)), max(abs(yl)), 100))','r','LineWidth',2);
ylim([0 100]);
legend('theta', 'theta cmd', 'q', 'Motor 1 (hinten rechts)',...
        'Motor 2 (vorne rechts)', 'Motor 3 (hinten links)',...
        'Motor 4 (vorne links)')
xlabel('t [s]')

subplot(2,1,2)
title('Rollregler')
yyaxis left;
plot(att_time,euler(:,3))
hold on
plot(cmd(:,4),cmd(:,1))
plot(imu_time,rates(:,1))
ylabel('Winkel [??], Drehrate [??/s]')
% yl = ylim;
% ylim([-max(abs(yl)) max(abs(yl))]);
ylim([-100 100])
yyaxis right;
plot(thrusts(:,7),thrusts(:,5))
hold on
plot(thrusts(:,7),thrusts(:,6))
ylim([0 100]);
plot(t_regler_start.*ones(100,1), (linspace(-max(abs(yl)), max(abs(yl)), 100))','g','LineWidth',2);
plot(t_regler_stop.*ones(100,1), (linspace(-max(abs(yl)), max(abs(yl)), 100))','r','LineWidth',2);
legend('phi', 'phi cmd', 'p', 'Motor 5 (rechts)',...
        'Motor 6 (links)')
xlabel('t [s]')

%% plot thrusts
figure
if(length(mtr_msgs) > 10) % regler war aktiv, auf gleiche zeitachse bringen
    subplot(2,1,1) 
    plot(thrusts(:,7),thrusts(:,1))
    hold on
    plot(thrusts(:,7),thrusts(:,2))
    plot(thrusts(:,7),thrusts(:,3),'--')
    plot(thrusts(:,7),thrusts(:,4),'--')
    plot(thrusts(:,7),thrusts(:,5))
    plot(thrusts(:,7),thrusts(:,6))
    plot(cmd(:,4),cmd(:,3),'v')
    plot(t_regler_start.*ones(100,1), (linspace(-max(abs(yl)), max(abs(yl)), 100))','g','LineWidth',2);
    plot(t_regler_stop.*ones(100,1), (linspace(-max(abs(yl)), max(abs(yl)), 100))','r','LineWidth',2);
    title('Regler Thrusts [% of full throttle]; Hex Clean Flight Numbering')
    ylim([0 100]);
    [~,hObj]=legend('Motor 1','Motor 2','Motor 3','Motor 4','Motor 5','Motor 6',...
                    'Throttle dT cmd');% return the handles array
    hL=findobj(hObj,'type','line');  % get the lines, not text
    set(hL,'linewidth',2)
    xlabel('t [s]')
    thrusts_fc(:,7) = thrusts_fc(:,7) - ones(size(thrusts_fc(:,7))).*(thrusts_fc(end,7)-thrusts(end,7));
end

subplot(2,1,2)
plot(thrusts_fc(:,7),thrusts_fc(:,1))
hold on
plot(thrusts_fc(:,7),thrusts_fc(:,2))
plot(thrusts_fc(:,7),thrusts_fc(:,3))
plot(thrusts_fc(:,7),thrusts_fc(:,4))
plot(thrusts_fc(:,7),thrusts_fc(:,5))
plot(thrusts_fc(:,7),thrusts_fc(:,6))
ylim([0 100]);
plot(t_regler_start.*ones(100,1), (linspace(-max(abs(yl)), max(abs(yl)), 100))','g','LineWidth',2);
plot(t_regler_stop.*ones(100,1), (linspace(-max(abs(yl)), max(abs(yl)), 100))','r','LineWidth',2);
title('Thrusts FC [% of full throttle]; Hex Clean Flight Numbering')
[~,hObj]=legend('Motor 1','Motor 2','Motor 3','Motor 4','Motor 5','Motor 6');           % return the handles array
hL=findobj(hObj,'type','line');  % get the lines, not text
set(hL,'linewidth',2)
xlabel('t [s]')

%% imu ACC
figure
subplot(2,1,1)
plot(imu_time,acc(:,1))
hold on
plot(imu_time,acc(:,2))
plot(imu_time,acc(:,3))
title('Beschleunigungen Body Frame [m/s^2]')
legend('X', 'Y', 'Z')
xlabel('t [s]')

subplot(2,1,2)
plot(imu_time,acc_NED(:,1))
hold on
plot(imu_time,acc_NED(:,2))
plot(imu_time,acc_NED(:,3))
title('Beschleunigungen XY Body horizontal Z down Frame [m/s^2]')
legend('X', 'Y', 'Z')
xlabel('t [s]')

% figure
% plot(imu_time,v(:,1))
% hold on
% plot(imu_time,v(:,2))
% plot(imu_time,v(:,3))
% title('Geschwindigkeiten XY Body horizontal Z down Frame [m/s]')
% legend('X', 'Y', 'Z')
% xlabel('t [s]')

%% Height
figure
subplot(3,1,1)
%alt_lid_time = (linspace(0,alt(end,1),length(h_lid)))';
plot(alt_lid_time(:,1),h_lid_raw(:,1))
hold on
plot(alt_lid_time(:,1),h_filter(:,1))
title('Lidar Altitude [m]')
xlabel('t [s]')
legend('raw','filtered')
subplot(3,1,2)
plot(alt_fc_time, h_fc(:,1))
title('FC Altitude')
subplot(3,1,3)
plot(alt_fc_time, h_fc(:,2))
title('FC h_dot (Variometer)')

figure
plot(imu_time,euler_imu(:,1))
hold on
plot(imu_time,euler_imu(:,2))
plot(att_time,euler(:,3))
plot(att_time,euler(:,2))
%plot(imu_time,euler_imu(:,3))
title('Eulerwinkel aus IMU integriert [??]')
%legend('phi', 'theta', 'psi')
legend('phi integriert', 'theta integriert', 'phi filter', 'theta filter')
xlabel('t [s]')

% figure
% plot(rc_time,rc_roll);
% hold on
% plot(rc_time,rc_pitch);
% plot(rc_time,rc_yaw);
% plot(rc_time,rc_T);
% legend('roll','pitch', 'yaw', 'throttle')
% 
% figure
% plot(rc_pilot_time,rc_pilot_roll);
% hold on
% plot(rc_pilot_time,rc_pilot_pitch);
% plot(rc_pilot_time,rc_pilot_yaw);
% plot(rc_pilot_time,rc_pilot_T);
% legend('roll','pitch', 'yaw', 'throttle')