%===============================================================
%  nao_matlab.m  (Webots MATLAB Controller)
%===============================================================
function nao_matlab
  
% YOLO detector 불러오기 (한 번만)
persistent yoloDet dashFig lidarFig lidarAx ...
           obAgent mdlLoaded sampleTime ...
           planState fwdCnt forwardBatch ...
           robot_pos stepFwd stepLat planOut iterInPlan
           

if isempty(yoloDet)
    yoloFile = 'detector_nao.mat';            % 경로 필요하면 수정
    fprintf("Loading YOLO detector ...\n");
    yoloData = load(yoloFile);
    yoloDet  = yoloData.detector;             % yolov4ObjectDetector
    fprintf("YOLO detector loaded.\n");
end

if isempty(dashFig) || ~isgraphics(dashFig)
    dashFig = figure('Name','Dashboard','NumberTitle','off');
end

save_images = struct('camera_top', true, 'camera_top_r', true, 'camera_top_l', true);

if isempty(obAgent)
    load('trainedObstacleAvoidanceAgent_250618.mat','obstacleAvoidanceAgent');
    obAgent = obstacleAvoidanceAgent;                       % 캐시
    assignin('base','obstacleAvoidanceAgent',obAgent);      % 베이스 등록
end

if isempty(mdlLoaded)
    mdl = 'AOMR_reward_goal';
    load_system(mdl);                                       % 창 안 띄움
    set_param([mdl '/Agent'],'Agent','obstacleAvoidanceAgent');
    sampleTime = 0.1;                                       % ← 고정 샘플타임
    assignin('base','sampleTime',sampleTime);
    mdlLoaded = true;
end

assignin('base','mapScale'  ,1);
scanAngles = -3*pi/8:pi/8:3*pi/8;
assignin('base','scanAngles'        ,scanAngles);
assignin('base','maxRange'  ,12);

assignin('base','initX'     ,10);
assignin('base','initY'     ,2);
assignin('base','initTheta' ,pi/2);
assignin('base','maxLinSpeed'     ,0.3);
assignin('base','maxAngSpeed'     ,0.3);

lidarNoiseVariance = 0.1^2;
lidarNoiseSeeds    = randi(intmax,size(scanAngles));   % 크기 맞춤
assignin('base','lidarNoiseVariance',lidarNoiseVariance);
assignin('base','lidarNoiseSeeds'   ,lidarNoiseSeeds);

nc = 20; nl = 26;                       % 맵 크기 (열×행)

binMap = createBinaryMap(nl,nc,[],[3 4]);
show(binaryOccupancyMap(binMap)); title('Occupancy Map');
assignin('base','mapMatrix' ,binMap);
simOut = sim(mdl, ...
             'StopTime','200', ...
             'SrcWorkspace','base'); 

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
SideStepLeft = wbu_motion_new('../../motions/SideStepLeft.motion');
SideStepRight = wbu_motion_new('../../motions/SideStepRight.motion');

% initialize motion
current_motion = forwards_motion;
% wbu_motion_set_loop(current_motion, false);
% wbu_motion_play(current_motion);

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
                  'camera_top_l', fullfile(base_folder, 'top_l'),...
                  'lidar', fullfile(base_folder, 'lidar'));

create_folder_if_not_exists(folders.camera_top);
create_folder_if_not_exists(folders.camera_top_r);
create_folder_if_not_exists(folders.camera_top_l);
create_folder_if_not_exists(folders.lidar);


%% ──────────────────────────────────────────────────────────────
% 4. 경로-계획 & 이동 상태 변수
%%% === UPDATE ===
if isempty(planState)
    planState   = "PLAN";  % PLAN → MOVE
    fwdCnt      = 0;       % 누적 전진 횟수
    forwardBatch= 4;       % 4회마다 재-계획
    stepFwd     = 1;       % 1 [m] 전진
    stepLat     = 0.7;     % 0.7 [m] 좌/우
    robot_pos   = [10,2];  % 시뮬레이션 상 로봇 좌표
    iterInPlan  = 0;
    planOut     = [];      % Simulink 결과 저장
end

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
            % 저장할 때 주석해제
            % fileName = sprintf('lidar_%06d.csv', steps);      % 000001, 000002 … 
            % writematrix(rangeMat, fullfile(folders.lidar,fileName));
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
  if mod(steps,1) == 0
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
  [left,right] = computeFootPressure(fsv,steps);

  % plot foot pressures
  subplot(2,4,5,'Parent',dashFig);
  plotFootPressure(left,right,steps,TIME_STEP);

  % plot accelerometer values
  acc = wb_accelerometer_get_values(accelerometer);
  subplot(2,4,6,'Parent',dashFig);
  bar([1:3], acc);
  title('Accelerometers');
  xlabel('Axes X Y Z');
  ylabel('Acceleration [m/s^2]');
  axis([0.5 3.5 -5 15]);

    %% 4-5. LiDAR Polar Plot (별도 창) ─────────────────────────────────
    [lidarFig,lidarAx, lidar_param] = updateLidarPolar(rangeMat,lidar,lidarFig,lidarAx);

    if planState == "PLAN" && ~isempty(rangeMat)
        % (1) Occupancy Map 작성 (DBSCAN)
        [xyHits,binMap] = lidar2OccMap(rangeMat,lidar_param,nl,nc);
        if ~isempty(xyHits)
            assignin('base','mapMatrix',binMap);

            % (2) Simulink RL Path 생성
            simOut = sim(mdl,'StopTime','200','SrcWorkspace','base');
            planOut = simOut.pose;          % N×3  (x y theta)
            iterInPlan = 0;                 % move 루프 인덱스
            planState  = "MOVE";            % 다음 스텝부터 이동
            subplot(2,4,7);
            show(binaryOccupancyMap(binMap)); title('Occupancy Map');
            subplot(2,4,8);
            plot(planOut(:,1),planOut(:,2)); title('RL Path');
        end

    %-----------------------------------------------------------
    % MOVE 상태 : planOut 경로를 따라 1 m 전진 × 4
    %-----------------------------------------------------------
    elseif planState == "MOVE" && ~isempty(planOut)
        iterInPlan = iterInPlan + 1;

        % (a) 목표 y 설정 (1 m 앞)
        targetY = robot_pos(2) + stepFwd;

        % (b) 경로 중 가장 근접한 y 값 찾기
        [~,idx] = min(abs(planOut(:,2) - targetY));
        targetX = planOut(idx,1);

        % (c) 좌/우 스텝 수 계산
        deltaX   = targetX - robot_pos(1);
        nLatStep = round(deltaX/stepLat);
        moveLat  = nLatStep * stepLat;

        % (d) **모션 실행**  (좌우 → 전진)
        if nLatStep > 0
            for i = 1:nLatStep
                wbu_motion_play(SideStepLeft);
                while ~wbu_motion_is_over(SideStepLeft)
                    wb_robot_step(TIME_STEP);
                end
            end
        elseif nLatStep < 0
            for i = 1:abs(nLatStep)
                wbu_motion_play(SideStepRight);
                while ~wbu_motion_is_over(SideStepRight)
                    wb_robot_step(TIME_STEP);
                end
            end
        end
        % 전진 1 m
        wbu_motion_play(forwards_motion);
        while ~wbu_motion_is_over(forwards_motion)
            wb_robot_step(TIME_STEP);
        end

        % (e) 시뮬레이션 좌표 갱신
        robot_pos = robot_pos + [moveLat, stepFwd];
        fprintf('[Move %02d] ΔX=%.2f → Lat=%d, Fwd=1 m | pos=(%.2f,%.2f)\n',...
                iterInPlan,deltaX,nLatStep,robot_pos);

        % (f) 누적 전진 횟수 체크
        fwdCnt = fwdCnt + 1;
        if iterInPlan >= forwardBatch  % 4회 완료
            planState  = "PLAN";       % 다시 경로 계획
        end
    end
%%
  drawnow;

  
end
