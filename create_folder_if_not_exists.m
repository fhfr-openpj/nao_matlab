function create_folder_if_not_exists(folder_path)
    if ~exist(folder_path, 'dir')
        mkdir(folder_path); % 폴더가 없으면 생성
    end
end