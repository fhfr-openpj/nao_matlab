% Description: MATLAB controller example for Webots
function nao_matlab
  
persistent lidarFig lidarAx

TIME_STEP = 40;

% --- Lidar -------------------------------------------------------------
%%%% ---- LIDAR ADD ----
lidar = wb_robot_get_device('lidar');             % Webots world에서 노드 이름과 일치
wb_lidar_enable(lidar, TIME_STEP);                % range image 모드
% 필요한 경우 포인트 클라우드를 쓰려면 아래 주석 해제
% wb_lidar_enable_point_cloud(lidar);
rangeMat = []
%%%% --------------------------------------------------------------------

% load motions
forwards_motion = wbu_motion_new('../../motions/Forwards.motion');
backwards_motion = wbu_motion_new('../../motions/Backwards.motion');
turn_left_motion = wbu_motion_new('../../motions/TurnLeft40.motion');
turn_right_motion = wbu_motion_new('../../motions/TurnRight40.motion');

% initialize motion
current_motion = forwards_motion;
wbu_motion_set_loop(current_motion, true);
wbu_motion_play(current_motion);

% get and enable receiver
receiver = wb_robot_get_device('receiver');
wb_receiver_enable(receiver,TIME_STEP);

% count the steps
steps = 0;

% 폴더 경로 설정

lidarDataDir = './dataset/lidar';
if ~exist(lidarDataDir,'dir')
    mkdir(lidarDataDir);
end


while wb_robot_step(TIME_STEP) ~= -1

  steps = steps + 1;
  
  % receive text messages from Supervisor
  while wb_receiver_get_queue_length(receiver) > 0
    message = wb_receiver_get_data(receiver,'string');
    disp(['Received message: "' message '"']);
    wb_receiver_next_packet(receiver);
  end

  % get camera RGB image
  % this function return an image in true color (RGB) uint8 format

    % Lidar range image
    if mod(steps, 20) == 0
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
    end

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
