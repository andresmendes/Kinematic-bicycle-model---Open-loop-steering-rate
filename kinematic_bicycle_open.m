%% Kinematic bicycle model - Open loop steering rate
% Simulation and animation of a kinematic bicycle model with open loop 
% steering rate making left and right turns.
%
%% 

clear ; close all ; clc

%% Scenario 

% Car
L = 2.7;                        % Wheelbase                     [m]
a = 0.935;                      % Front overhang                [m]
b = L/2;                        % Dist. CG - front axle         [m]
c = L/2;                        % Dist. CG - rear axle          [m]
d = 0.995;                      % Rear overhang                 [m]
w = 1.780;                      % Width                         [m]

% Initial conditions 
x0      = 0;                    % Initial x rear axle           [m]
y0      = 0;                    % Initial y rear axle           [m]
psi0    = 0;                    % Initial yaw angle             [rad]
delta0  = 0;                    % Initial steering angle        [rad]
z0 = [x0 x0 psi0 delta0];

% Parameters
tf      = 30;                   % Final time                    [s]
fR      = 30;                   % Frame rate                    [fps]
dt      = 1/fR;                 % Time resolution               [s]
time    = linspace(0,tf,tf*fR); % Time                          [s]

%% Simulation   
options = odeset('RelTol',1e-5);
[tout,zout] = ode45(@(t,z) car(t,z,L),time,z0,options);

% Retrieving states
x = zout(:,1);                  % Rear axle x position          [m]
y = zout(:,2);                  % Rear axle y position          [m]
g = zout(:,3);                  % Yaw angle                     [rad]
% delta = zout(:,4);

% Yaw rate and speed
% Preallocating
dg  = zeros(length(time),1);
v   = zeros(length(time),1);
for i=1:length(time)
    [dz,vel]    = car(time(i),zout(i,:),L);
    dg(i)       = dz(3);
    v(i)        = vel;
end

% States for animation
XT      = x + c*cos(g);         % CG X location                 [m]
YT      = y + c*sin(g);         % CG Y location                 [m]
PSI     = zout(:,3);            % Yaw angle                     [rad]
dPSI    = dg;                   % Yaw rate                      [rad/s]
VEL     = v;                    % Vehicle speed                 [m/s]
ALPHAT  = atan(dg*c/v);         % Vehicle side slip angle       [rad]

%% Animation

% Struct for animation
data.XT        = XT;
data.YT        = YT;
data.PSI       = PSI;
data.dPSI      = dPSI;
data.VEL       = VEL;
data.ALPHAT    = ALPHAT;
data.TSpan     = time;
data.a         = a;
data.b         = b;
data.c         = c;
data.d         = d;
data.wT        = w;

animation(data);

%% Auxiliary functions

function [dz,v] = car(t,z,L)

    % States
%     x = z(1);                   % x position of rear axle       [m]
%     y = z(2);                   % y position of rear axle       [m]
    g = z(3);
    delta = z(4);               % Steering angle                [rad] 

    v = 15/3.6;                 % Speed of rear axle            [m/s]
    
    % STEERING
    % Straight line
        t1 = 2;                 % Straight line duration        [s]
        deltaRate = 0;
    % Steering change 1
        steerChangeDuration_1 = 1; % [s]
        if t > t1 && t < t1 + steerChangeDuration_1
            deltaRate = 15*pi/180;
        end
    % Steering change 2
        t2 = 12;
        steerChangeDuration_2 = 2; % [s]
        if t > t2 && t < t2 + steerChangeDuration_2
            deltaRate = -15*pi/180;
        end
    % Steering change 3
        t3 = 23;
        steerChangeDuration_3 = 1; % [s]
        if t > t3 && t < t3 + steerChangeDuration_3
            deltaRate = 15*pi/180;
        end
    
    % Kinematic bicycle model
    dx = v*cos(g);
    dy = v*sin(g);
    dg = v/L*tan(delta);
    dd = deltaRate; 

    dz = [dx ; dy ; dg ; dd];
    
end

function animation(data)

    % Color
    color_Front_Axle    = 'b';
    color_Rear_Axle     = 'g';
    color_Car           = 'r';

    % Retrieving data
    TOUT    = data.TSpan;
    XT      = data.XT;              % CG x position             [m]
    YT      = data.YT;              % CG y position             [m]
    PSI     = data.PSI;             % Vehicle yaw angle         [rad]
    VEL     = data.VEL;             % Vehicle CG velocity       [m/s]
    ALPHAT  = data.ALPHAT;          % Vehicle side slip angle   [rad]
    dPSI    = data.dPSI;            % Yaw rate                  [rad/s]
    a       = data.a;               % Front overhang            [m]
    b       = data.b;               % Distance FT               [m]
    c       = data.c;               % Distance TR               [m]
    d       = data.d;               % Rear overhang             [m]
    lT      = data.wT / 2;          % Half width of the vehicle [m]

    % Slip angle @ front axle [rad]
    ALPHAF = atan2((b * dPSI + VEL.*sin(ALPHAT)),(VEL.*cos(ALPHAT)));
    % OBS: No steering angle because it measures the angle between velocity
    % vector and longitudinal axle of the vehicle.
    
    %Slip angle @ rear axle [rad]
    ALPHAR = atan2((-c * dPSI + VEL.*sin(ALPHAT)),(VEL.*cos(ALPHAT)));

    % Speed @ front axle [m/s]
    VF = sqrt((VEL.*cos(ALPHAT)).^2 + (b * dPSI + VEL.*sin(ALPHAT)).^2);
    % Speed @ rear axle [m/s]
    VR = sqrt((VEL.*cos(ALPHAT)).^2 + (-c * dPSI + VEL.*sin(ALPHAT)).^2);

    % Position of the corners and axles relative to CG
    % Position vectors 1, 2, 3 e 4 relative to T base (T t1 t2 t3)
    rt1t = [a+b;lT];                    % Front left
    rt2t = [a+b;-lT];                   % Front right
    rt3t = [-c-d;-lT];                  % Rear right
    rt4t = [-c-d;lT];                   % Rear left

    eif = [b;0];                        % Front axle
    eir = [-c;0];                       % Rear axle

    % Absolute position of corners and axles
    % Movement of the points from change of orientation.

    % Preallocating matrix
    rt1i = zeros(length(TOUT),2);
    rt2i = zeros(length(TOUT),2);
    rt3i = zeros(length(TOUT),2);
    rt4i = zeros(length(TOUT),2);

    eff = zeros(length(TOUT),2);
    err = zeros(length(TOUT),2);

    % Loop start
    for j=1:length(TOUT)
        % Rotation matrix base (T t1 t2 t3) to (o i j k)
        RTI=[cos(PSI(j)) -sin(PSI(j));sin(PSI(j)) cos(PSI(j))];
        % Position vectors 1, 2, 3 e 4 relative to origin of the inertial
        % reference base (T t1 t2 t3)
        rt1i(j, 1:2) = (RTI * rt1t)';
        rt2i(j, 1:2) = (RTI * rt2t)';
        rt3i(j, 1:2) = (RTI * rt3t)';
        rt4i(j, 1:2) = (RTI * rt4t)';
        % Position of front and rear axle
        eff(j, 1:2) = (RTI * eif);     % Front
        err(j, 1:2) = (RTI * eir);     % Rear
    end

    % Absolute position of corners and axles
    % Position vectors 1, 2, 3 e 4 relative to o base (o i j k)
    rc1t=[XT YT]+rt1i;
    rc2t=[XT YT]+rt2i;
    rc3t=[XT YT]+rt3i;
    rc4t=[XT YT]+rt4i;
    % Absolute position of the front and rear axle
    ef = [XT YT]+eff;
    er = [XT YT]+err;

    figure
    % set(gcf,'Position',[50 50 1280 720]) % YouTube: 720p
    % set(gcf,'Position',[50 50 854 480]) % YouTube: 480p
    set(gcf,'Position',[50 50 640 640]) % Social
    
    % Create and open video writer object
    v = VideoWriter('Kinematic_bicycle_open.mp4','MPEG-4');
    v.Quality = 100;
    open(v);
    
    hold on ; grid on ; axis equal
    set(gca,'xlim',[min(XT)-5 max(XT)+5],'ylim',[min(YT)-5 max(YT)+5])
    xlabel('x distance [m]');
    ylabel('y distance [m]');

    for j = 1:length(TOUT)
        
        cla
        
        % Axles
        plot(ef(:,1),ef(:,2),color_Front_Axle)
        plot(er(:,1),er(:,2),color_Rear_Axle)

        % Coordinates of the corners
        xc = [rc1t(j, 1) rc2t(j, 1) rc3t(j, 1) rc4t(j, 1)];
        yc = [rc1t(j, 2) rc2t(j, 2) rc3t(j, 2) rc4t(j, 2)];

        % Vehicle
        fill(xc, yc,color_Car)

        % Velocity vectors
        % Different colors
        vector(ef(j, 1:2),(ALPHAF(j)+PSI(j)),VF(j),color_Front_Axle);
        vector(er(j, 1:2),(ALPHAR(j)+PSI(j)),VR(j),color_Rear_Axle);

        title(strcat('Time=',num2str(TOUT(j),"%.2f"),' s'))

        frame = getframe(gcf);
        writeVideo(v,frame);

    end

    close(v);
    
end

function vector(startCoord, angle, modulus, color)

    coord1  = startCoord;                                   % Vector start coordinate
    theta   = angle;
    modulus = 0.7 * modulus;                                % Size of the vector
    coord2  = modulus*[cos(theta) sin(theta)] + coord1;     % Vector end coordinate

    %theta = atan2((coord1(1)-coord2(1)),(coord1(2)-coord2(2))); % Orientatin angle of the triangle
    esc     = 1;    % Scale
    l       = 0.5;  % width relative to the triangle length (0-1)

    % Shape and orientation of the triangle
    c1 = esc * l*[-sin(theta) +cos(theta)];     % corner 1 - bottom left
    c2 = esc * l*[+sin(theta) -cos(theta)];     % corner 2 - bottom right
    c3 = esc*[+cos(theta) +sin(theta)];         % corner 3 - top central

    % Scale and positioning
    x = [c1(1)+coord2(1) c2(1)+coord2(1) c3(1)+coord2(1)];
    y = [c1(2)+coord2(2) c2(2)+coord2(2) c3(2)+coord2(2)];

    hold on
    fill(x, y, color)
    p = plot([coord1(1) coord2(1)],[coord1(2) coord2(2)],color);
    set(p, 'LineWidth', 2)

    % Vector origin
    m = plot(coord1(1),coord1(2),strcat('*', color));
    set(m, 'MarkerSize', 10)
    
end
