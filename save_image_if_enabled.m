function save_image_if_enabled(image_data, folder_path, save_enabled)
    if save_enabled
        disp("[Saving] " + folder_path);  % ← 확인용 로그
        % 기존 파일 확인 및 최소 번호 계산
        existing_files = dir(fullfile(folder_path, 'frame_*.png'));
        existing_numbers = [];
        for k = 1:length(existing_files)
            file_name = existing_files(k).name;
            match = regexp(file_name, 'frame_(\d+)\.png', 'tokens');
            if ~isempty(match)
                existing_numbers(end+1) = str2double(match{1}{1});
            end
        end

        if isempty(existing_numbers)
            next_number = 1; % 파일이 없으면 1부터 시작
        else
            next_number = min(setdiff(1:max(existing_numbers)+1, existing_numbers));
        end

        % 이미지 저장
        img_filename = sprintf('%s/frame_%d.png', folder_path, next_number);
        imwrite(image_data, img_filename);
        fprintf("[Saved] %s\n", img_filename);   % ← 확인용 로그
    end
end