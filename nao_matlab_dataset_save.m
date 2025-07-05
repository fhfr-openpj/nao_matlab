function nao_matlab_dataset

%% ── 기본 설정 ────────────────────────────────────────────────
TIME_STEP = 40;                       % [ms]


%% ── 디바이스 핸들 ─────────────────────────────────────────────
camera        = wb_robot_get_device('CameraTop');
wb_camera_enable(camera, TIME_STEP);

% 모션 파일 : 좌 · 우 · 전진
forwards_motion   = wbu_motion_new('../../motions/Forwards.motion');
SideStepLeft      = wbu_motion_new('../../motions/SideStepLeft.motion');
SideStepRight     = wbu_motion_new('../../motions/SideStepRight.motion');

% 루프 재생 해제(한 번씩만 실행할 것이므로)
wbu_motion_set_loop(forwards_motion,   false);
wbu_motion_set_loop(SideStepLeft,      false);
wbu_motion_set_loop(SideStepRight,     false);

% Supervisor→Robot 수신 (필요 시)
receiver = wb_robot_get_device('receiver');
wb_receiver_enable(receiver, TIME_STEP);

%% ── 데이터셋 저장용 폴더 ──────────────────────────────────────
base_folder = './dataset/img';
folders     = struct('camera_top', fullfile(base_folder,'top_tmp'));
create_folder_if_not_exists(folders.camera_top);
save_rgb = true;    % CameraTop 저장 여부

%% ── 모션 시퀀스용 상태 변수 ──────────────────────────────────
persistent current_motion motion_idx
if isempty(motion_idx)
    motion_idx     = 1;           % 1:L → 2:R → 3:F
    current_motion = SideStepLeft;
    wbu_motion_play(current_motion);
end

%% ── 실행 루프 ────────────────────────────────────────────────
steps = 0;
while wb_robot_step(TIME_STEP) ~= -1
    steps = steps + 1;

    % ── Supervisor 메시지 체크(선택) ─────────────────────────
    while wb_receiver_get_queue_length(receiver) > 0
        msg = wb_receiver_get_data(receiver,'string');
        disp(['Supervisor: "' msg '"']);
        wb_receiver_next_packet(receiver);
    end

    % ── 카메라 프레임 취득 & 저장 ────────────────────────────
    % ① 카메라 프레임 취득
    rgb = wb_camera_get_image(camera);

    % ② 데이터셋 저장
    % ▶ 매 프레임 PNG로 저장 (필요하면 mod(steps, N)==0 조건 사용)
    if mod(steps, 15) == 0
      save_image_if_enabled(rgb,  folders.camera_top, save_rgb);
      disp(['Saved frame #' num2str(steps) ' to ' folders.camera_top]);
    end

    % ── 시각화(선택 사항) ───────────────────────────────────
    if mod(steps, 1) == 0      % 0.4 s 마다 한 번만 그림
        subplot(2,2,1); image(rgb);  title('RGB Camera');
        drawnow;
    end

    % ── 모션 시퀀스 제어 ─────────────────────────────────────
    if wbu_motion_is_over(current_motion)
        % 다음 모션 인덱스
        motion_idx = motion_idx + 1;
        if motion_idx > 4, motion_idx = 1; end

        switch motion_idx
            case 1
                current_motion = SideStepLeft;
            case 2
                current_motion = SideStepRight;
            case 3
                current_motion = SideStepLeft;
            case 4
                current_motion = forwards_motion;
        end
        wbu_motion_play(current_motion);
    end
end
end   % ── function nao_matlab_dataset ─────────────────────────
