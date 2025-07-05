% Description: MATLAB controller example for Webots
function nao_matlab
  
% YOLO detector 불러오기 (한 번만)
persistent yoloDet
if isempty(yoloDet)
    yoloFile = 'detector_nao.mat';            % 경로 필요하면 수정
    fprintf("Loading YOLO detector ...\n");
    yoloData = load(yoloFile);
    yoloDet  = yoloData.detector;             % yolov4ObjectDetector
    fprintf("YOLO detector loaded.\n");
end

persistent dashFig lidarFig lidarAx
if isempty(dashFig) || ~isgraphics(dashFig)
    dashFig = figure('Name','Dashboard','NumberTitle','off');
end


save_images = struct('camera_top', true, 'camera_top_r', true, 'camera_top_l', true);

% uncomment the next two lines if you want to use
% MATLAB's desktop and interact with the controller
%desktop;
%keyboard;

% control step
TIME_STEP = 40;

% Laplacian edge detection matrix
conv_matrix = [
 -1 -1 -1;
 -1  8 -1;
 -1 -1 -1 ];

% get and enable camera
camera = wb_robot_get_device('CameraTop');
wb_camera_enable(camera,TIME_STEP);

camera_top_r = wb_robot_get_device('CameraTop R');
camera_top_l = wb_robot_get_device('CameraTop L');
wb_camera_enable(camera_top_r, TIME_STEP);
wb_camera_enable(camera_top_l, TIME_STEP);

% --- Lidar -------------------------------------------------------------
%%%% ---- LIDAR ADD ----
lidar = wb_robot_get_device('lidar');             % Webots world에서 노드 이름과 일치
wb_lidar_enable(lidar, TIME_STEP);                % range image 모드
% 필요한 경우 포인트 클라우드를 쓰려면 아래 주석 해제
% wb_lidar_enable_point_cloud(lidar);
%%%% --------------------------------------------------------------------


% get and enable accelerometer
accelerometer = wb_robot_get_device('accelerometer');
wb_accelerometer_enable(accelerometer,TIME_STEP);

% load motions
forwards_motion = wbu_motion_new('../../motions/Forwards.motion');
backwards_motion = wbu_motion_new('../../motions/Backwards.motion');
turn_left_motion = wbu_motion_new('../../motions/TurnLeft40.motion');
turn_right_motion = wbu_motion_new('../../motions/TurnRight40.motion');

% initialize motion
current_motion = forwards_motion;
wbu_motion_set_loop(current_motion, true);
wbu_motion_play(current_motion);

% get and enable foot sensors
fsr(1) = wb_robot_get_device('LFsr');
fsr(2) = wb_robot_get_device('RFsr');
wb_touch_sensor_enable(fsr(1), TIME_STEP);
wb_touch_sensor_enable(fsr(2), TIME_STEP);

% get and enable receiver
receiver = wb_robot_get_device('receiver');
wb_receiver_enable(receiver,TIME_STEP);

% count the steps
steps = 0;

% 폴더 경로 설정
base_folder = './resource/img';
folders = struct('camera_top', fullfile(base_folder, 'top'), ...
                  'camera_top_r', fullfile(base_folder, 'top_r'), ...
                  'camera_top_l', fullfile(base_folder, 'top_l'));

lidarDataDir = './resource/data/lidar';
if ~exist(lidarDataDir,'dir')
    mkdir(lidarDataDir);
end

create_folder_if_not_exists(folders.camera_top);
create_folder_if_not_exists(folders.camera_top_r);
create_folder_if_not_exists(folders.camera_top_l);

while wb_robot_step(TIME_STEP) ~= -1

  figure(dashFig);
  steps = steps + 1;
  
  % receive text messages from Supervisor
  while wb_receiver_get_queue_length(receiver) > 0
    message = wb_receiver_get_data(receiver,'string');
    disp(['Received message: "' message '"']);
    wb_receiver_next_packet(receiver);
  end

  % get camera RGB image
  % this function return an image in true color (RGB) uint8 format
    % ---------- 4-2. 카메라 이미지 ----------
    rgb        = wb_camera_get_image(camera);
    rgb_top_r  = wb_camera_get_image(camera_top_r);
    rgb_top_l  = wb_camera_get_image(camera_top_l);

    % Lidar range image
    if steps > 1
        range = wb_lidar_get_range_image(lidar);
        if ~isempty(range)
            hRes   = wb_lidar_get_horizontal_resolution(lidar);
            nLayer = wb_lidar_get_number_of_layers(lidar);
            rangeMat = reshape(range, hRes, nLayer)';  % [layers × hRes]
            fileName = sprintf('lidar_%06d.csv', steps);      % 000001, 000002 …
            writematrix(rangeMat, fullfile(lidarDataDir,fileName));
        else
            rangeMat = [];
        end
    else
        rangeMat = [];
    end

  
  % display camera images
  subplot(2, 4, 1,'Parent',dashFig); % CameraTop R
  image(rgb_top_r);
  title('CameraTop R');

  subplot(2, 4, 2,'Parent',dashFig); % CameraTop L
  image(rgb_top_l);
  title('CameraTop L');

  % display camera image
  subplot(2,4,3,'Parent',dashFig);
    if mod(steps,1)==0
        [bboxes,scores,labels] = detect(yoloDet,rgb,Threshold=0.25);
        annStr = string(labels)+" : "+string(round(scores,2));
        rgbDisp = insertObjectAnnotation(rgb,"rectangle",bboxes,annStr);
    else
        rgbDisp = rgb;   % 검출 안 한 프레임은 원본 그대로
    end
  image(rgbDisp);
    % image(rgb);
  title('RGB Camera');

% every 10 time step
  if mod(steps,10) == 0
    % create intensity image from RGB image
    intens = double((rgb(:,:,1)+rgb(:,:,2)+rgb(:,:,3))/3);

    % edge detection
    edges = conv2(intens, conv_matrix, 'valid');

    % display edges image
    subplot(2,4,4,'Parent',dashFig);
    image(edges);
    colormap('gray');
    title('Edge detection');

    save_image_if_enabled(rgb,  folders.camera_top, save_images.camera_top);
    save_image_if_enabled(rgb_top_r,  folders.camera_top_r, save_images.camera_top_r);
    save_image_if_enabled(rgb_top_l,  folders.camera_top_l, save_images.camera_top_l);
  end

  % compute total foot pressures
  fsv(1,:) = wb_touch_sensor_get_values(fsr(1));
  fsv(2,:) = wb_touch_sensor_get_values(fsr(2));

  % The coefficients were calibrated against the real
  % robot so as to obtain realistic sensor values.
  l(1) = fsv(1,3)/3.4 + 1.5*fsv(1,1) + 1.15*fsv(1,2); % Left Foot Front Left
  l(2) = fsv(1,3)/3.4 + 1.5*fsv(1,1) - 1.15*fsv(1,2); % Left Foot Front Right
  l(3) = fsv(1,3)/3.4 - 1.5*fsv(1,1) - 1.15*fsv(1,2); % Left Foot Rear Right
  l(4) = fsv(1,3)/3.4 - 1.5*fsv(1,1) + 1.15*fsv(1,2); % Left Foot Rear Left

  r(1) = fsv(2,3)/3.4 + 1.5*fsv(2,1) + 1.15*fsv(2,2); % Right Foot Front Left
  r(2) = fsv(2,3)/3.4 + 1.5*fsv(2,1) - 1.15*fsv(2,2); % Right Foot Front Right
  r(3) = fsv(2,3)/3.4 - 1.5*fsv(2,1) - 1.15*fsv(2,2); % Right Foot Rear Right
  r(4) = fsv(2,3)/3.4 - 1.5*fsv(2,1) + 1.15*fsv(2,2); % Right Foot Rear Left

  left(steps) = 0;
  right(steps) = 0;
  for i = [1:4]
    l(i) = min(25, max(0, l(i)));
    r(i) = min(25, max(0, r(i)));
    left(steps) = left(steps) + l(i);
    right(steps) = right(steps) + r(i);
  end

  % plot foot pressures
  subplot(2,4,5,'Parent',dashFig);
  if steps <= 100
    time = [1:steps] * TIME_STEP / 1000;
    plot(time,left(),'b',time,right(),'r');
  else
    time = [steps-100:steps] * TIME_STEP / 1000;
    plot(time,left(steps-100:steps),'b',time,right(steps-100:steps),'r');
  end
  title('Left/right foot pressure');
  xlabel('time [s]');
  ylabel('Newtons [N]');

  % plot accelerometer values
  acc = wb_accelerometer_get_values(accelerometer);
  subplot(2,4,6,'Parent',dashFig);
  bar([1:3], acc);
  title('Accelerometers');
  xlabel('Axes X Y Z');
  ylabel('Acceleration [m/s^2]');
  axis([0.5 3.5 -5 15]);


%%
%% ---------- Lidar Polar Plot 업데이트 ----------
if isempty(lidarFig) || ~isgraphics(lidarFig)
    lidarFig  = figure('Name','Lidar Scan (polar)','NumberTitle','off');
    lidarAx   = axes('Parent',lidarFig);
    hold(lidarAx,'on');         
    axis(lidarAx,'equal'); grid(lidarAx,'on');
    xlabel(lidarAx,'X [m]'); ylabel(lidarAx,'Y [m]');
    lidarLines = gobjects(0);
    validPts   = gobjects(1);
end

if ~isempty(rangeMat)          % ── 유효 데이터 ──
    hRes   = wb_lidar_get_horizontal_resolution(lidar);
    nLayer = wb_lidar_get_number_of_layers(lidar);
    fov    = wb_lidar_get_fov(lidar);           % [rad]
    maxR   = wb_lidar_get_max_range(lidar);

    rValues   = rangeMat( ceil(nLayer/2), : );
    thetaVec  = linspace(  fov/2 , -fov/2 , hRes);

    xEnd = -rValues .* sin(thetaVec);
    yEnd =  rValues .* cos(thetaVec);

    %% -------- [수정 1] 이전 빔 지우기/재사용 -----------
    needNew = isempty(lidarLines) || numel(lidarLines)~=hRes || ...
          ~all(isgraphics(lidarLines));

    if isempty(lidarAx) || ~isgraphics(lidarAx)
        lidarAx = axes('Parent',lidarFig);
        hold(lidarAx,'on');                 % ★
        axis(lidarAx,'equal'); grid(lidarAx,'on');
        xlabel(lidarAx,'X [m]'); ylabel(lidarAx,'Y [m]');
    end    

    if needNew
        delete(lidarLines(isgraphics(lidarLines)));   % 기존 라인 완전히 삭제
        lidarLines = gobjects(1,hRes);      % ★ 빈 핸들 배열 새로 확보 ★
        for k = 1:hRes
            lidarLines(k) = line(lidarAx,[0 xEnd(k)],[0 yEnd(k)], ...
                     'Color',[0 0.6 1 0.6]);   % 마지막 0.6 = 알파값
        end
        % 외곽 원 한 번만
        t = linspace(0,2*pi,360);
        plot(lidarAx,maxR*cos(t),maxR*sin(t),'k:');
        title(lidarAx,'Lidar Polar Scan');
    else
        for k = 1:hRes
            set(lidarLines(k),'XData',[0 xEnd(k)],'YData',[0 yEnd(k)]);
        end
    end
    %% -----------------------------------------------

    %% -------- [수정 2] 실측 끝점 scatter --------------
    isHit = rValues < (maxR - 6);%eps;% 0.2;%              % maxR 바로 전 값 제외
    if ~isempty(validPts) && isgraphics(validPts)
        set(validPts,'XData',xEnd(isHit),'YData',yEnd(isHit));
    else
        validPts = scatter(lidarAx,xEnd(isHit),yEnd(isHit), ...
                           15,'filled','r');
    end
    %% -----------------------------------------------

    xlim(lidarAx,[-maxR maxR]); ylim(lidarAx,[-maxR maxR]);

    disp([num2str(sum(isgraphics(lidarLines))),' beams drawn.']);

else                            % ── 워밍업 ──
    cla(lidarAx);
    title(lidarAx,'Lidar Scan (warming up)');
    lidarLines = gobjects(0); validPts = gobjects(1);
end


%%
  if mod(steps,10) == 0
    %코드가 추가될 부분

  end

%% ----------------------------------------------------
% xy_hits = [xEnd(isHit).' , yEnd(isHit).'];
%% ---------------------------------------------------------------
%%

  % flush graphics
  drawnow;

  
end
