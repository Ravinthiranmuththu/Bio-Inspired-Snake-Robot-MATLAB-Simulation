%% SnakeRobot_Physics_RevB_Corrected.m
% Quasi-static anisotropic-drag locomotion with 3D alternating yaw/pitch
% visualization. The planar body velocity is solved from force and yaw
% moment equilibrium; it is NOT a full 3D rigid-body/contact simulation.
% Requires base MATLAB (R2016b+ for local functions in scripts).
clear; close all; clc;

%% Parameters
N = 12; L = 0.18; nJ = N-1;
T = 18; fps = 24; dt = 1/fps; tvec = 0:dt:T;
A = deg2rad(32); f = 0.65; phase = 0.63; pitchA = deg2rad(7);
cParallel = 0.45; cNormal = 3.8; cYaw = 0.02;
exportVideo = false; videoName = 'Snake_Locomotion_RevB_Corrected.mp4';

% Pose of first-link origin: [x; y; heading].
pose = zeros(3,1);
% Columns: time, x, y, heading, vx, vy, yaw_rate.
history = zeros(numel(tvec),7);
trail = zeros(numel(tvec),2);

%% Graphics
fig = figure('Color','w','Name','Snake robot: friction-driven locomotion');
ax = axes('Parent',fig); hold(ax,'on'); grid(ax,'on'); axis(ax,'equal');
view(ax,35,25); xlabel(ax,'X (m)'); ylabel(ax,'Y (m)'); zlabel(ax,'Z (m)');
plot3(ax,[-5 15],[0 0],[0 0],':','Color',[.7 .7 .7]);
hLinks = gobjects(N,1); hJoints = gobjects(N+1,1);
for j=1:N
    hLinks(j) = plot3(ax,nan,nan,nan,'-','LineWidth',9, ...
        'Color',[.12 .50 .72]);
end
for j=1:N+1
    hJoints(j) = plot3(ax,nan,nan,nan,'o', ...
        'MarkerFaceColor',[.94 .65 .15], ...
        'MarkerEdgeColor','k','MarkerSize',6);
end
hTrail = plot3(ax,nan,nan,nan,'r-','LineWidth',1.4);

writer = [];
if exportVideo
    try
        writer = VideoWriter(videoName,'MPEG-4');
        writer.FrameRate = fps; open(writer);
    catch ME
        warning('Video export unavailable: %s',ME.message);
        exportVideo = false;
    end
end

%% Main simulation
for k=1:numel(tvec)
    t = tvec(k);
    [q,qd] = jointCommands(t,nJ,A,pitchA,f,phase);

    % Only yaw joints affect the simplified planar friction model.
    yaw = q; yawdot = qd;
    yaw(2:2:end) = 0;
    yawdot(2:2:end) = 0;
    [pLocal,vLocal,headingLocal] = planarGeometry(yaw,yawdot,L,N);

    % Solve instantaneous free-body velocity at CURRENT pose.
    R = rot2(pose(3));
    r = R*pLocal;
    vShape = R*vLocal;
    M = zeros(3,3); b = zeros(3,1);
    for j=1:N
        ang = pose(3)+headingLocal(j);
        tangent = [cos(ang);sin(ang)];
        normal = [-sin(ang);cos(ang)];
        D = cParallel*(tangent*tangent.') + ...
            cNormal*(normal*normal.');
        S = [1 0 -r(2,j); 0 1 r(1,j)];
        W = [1 0; 0 1; -r(2,j) r(1,j)];
        M = M + W*D*S;
        b = b + W*D*vShape(:,j);
    end
    M(3,3) = M(3,3)+cYaw;
    if rcond(M)<1e-12
        warning('Ill-conditioned drag matrix at t=%.3f s',t);
        bodyVel = -pinv(M)*b;
    else
        bodyVel = -(M\b);
    end

    % Record the pose and instantaneous velocity at the same time t.
    history(k,:) = [t, pose(:).', bodyVel(:).'];

    % Display 3D articulated geometry at the CURRENT pose.
    [xyz,~] = geometry3D(q,L,N);
    xyz(1:2,:) = R*xyz(1:2,:) + pose(1:2);
    trail(k,:) = xyz(1:2,1).';
    for j=1:N
        set(hLinks(j),'XData',xyz(1,j:j+1), ...
            'YData',xyz(2,j:j+1),'ZData',xyz(3,j:j+1));
    end
    for j=1:N+1
        set(hJoints(j),'XData',xyz(1,j), ...
            'YData',xyz(2,j),'ZData',xyz(3,j));
    end
    set(hTrail,'XData',trail(1:k,1), ...
        'YData',trail(1:k,2),'ZData',zeros(k,1));
    xlim(ax,[pose(1)-1.5 pose(1)+2.7]);
    ylim(ax,[pose(2)-2 pose(2)+2]);
    zlim(ax,[-0.5 0.7]);
    title(ax,sprintf('Quasi-static drag locomotion | t=%.1f s | X=%.2f m',t,pose(1)));
    drawnow;
    if exportVideo
        writeVideo(writer,getframe(fig));
    end

    % Advance to the NEXT time step (do not integrate after final sample).
    if k<numel(tvec)
        pose = pose + (tvec(k+1)-t)*bodyVel;
        pose(3) = atan2(sin(pose(3)),cos(pose(3)));
    end
end
if exportVideo
    close(writer);
end

%% Results
save('Snake_Physics_Results.mat','history','trail','N','L', ...
    'cParallel','cNormal','cYaw','A','f','phase','pitchA');
figure('Color','w','Name','Locomotion results');
subplot(3,1,1);
plot(history(:,1),history(:,2:3),'LineWidth',1.5); grid on;
ylabel('Position (m)'); legend('X','Y','Location','best');
title('Computed planar locomotion');
subplot(3,1,2);
plot(history(:,1),rad2deg(history(:,4)),'LineWidth',1.5); grid on;
ylabel('Heading (deg)');
subplot(3,1,3);
plot(history(:,1),history(:,5:6),'LineWidth',1.5); grid on;
ylabel('Velocity (m/s)'); xlabel('Time (s)');
legend('Vx','Vy','Location','best');
fprintf('Final first-link origin: X=%.3f m, Y=%.3f m, heading=%.1f deg\n', ...
    pose(1),pose(2),rad2deg(pose(3)));

%% Local functions
function [q,qd] = jointCommands(t,nJ,A,pitchA,f,phase)
    i = (1:nJ).'; w = 2*pi*f;
    amp = A*ones(nJ,1); amp(2:2:end) = pitchA;
    ramp = min(t/1.5,1);
    dramp = double(t<1.5)/1.5;
    arg = w*t-(i-1)*phase;
    q = ramp*amp.*sin(arg);
    qd = dramp*amp.*sin(arg) + ramp*amp*w.*cos(arg);
end

function [p,v,h] = planarGeometry(q,qd,L,N)
    h = zeros(N,1); hd = zeros(N,1);
    for i=2:N
        h(i) = h(i-1)+q(i-1);
        hd(i) = hd(i-1)+qd(i-1);
    end
    p = zeros(2,N); v = zeros(2,N);
    tip = [0;0]; tipv = [0;0];
    for i=1:N
        dir = [cos(h(i));sin(h(i))];
        normal = [-sin(h(i));cos(h(i))];
        p(:,i) = tip + (L/2)*dir;
        v(:,i) = tipv + (L/2)*hd(i)*normal;
        tip = tip+L*dir;
        tipv = tipv+L*hd(i)*normal;
    end
end

function R = rot2(theta)
    R = [cos(theta) -sin(theta);sin(theta) cos(theta)];
end

function [P,orient] = geometry3D(q,L,N)
    P = zeros(3,N+1); orient = eye(3);
    for i=1:N
        P(:,i+1) = P(:,i)+orient*[L;0;0];
        if i<N
            c = cos(q(i)); s = sin(q(i));
            if mod(i,2)==1
                Q = [c -s 0;s c 0;0 0 1];
            else
                Q = [c 0 s;0 1 0;-s 0 c];
            end
            orient = orient*Q;
        end
    end
end
