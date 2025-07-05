function binMap = createBinaryMap(nRows,nCols,obsRC,obsSizeRC)


M = size(obsRC,1);

% ---- obsSizeRC 규격 정규화 → M×2 행렬 [h v] ----
if isscalar(obsSizeRC)               % S           → S×S
    obsSizeRC = repmat([obsSizeRC obsSizeRC],M,1);
elseif isvector(obsSizeRC) && numel(obsSizeRC)==1
    obsSizeRC = repmat([obsSizeRC obsSizeRC],M,1);
elseif isvector(obsSizeRC) && numel(obsSizeRC)==2
    obsSizeRC = repmat(obsSizeRC(:).',M,1);   % 1×2 → M×2
elseif isequal(size(obsSizeRC),[M 1])
    obsSizeRC = [obsSizeRC obsSizeRC];        % M×1 → M×2(S,S)
else
    assert(isequal(size(obsSizeRC),[M 2]), ...
        "obsSizeRC must be scalar, M×1, 1×2, or M×2.");
end

% ---- 맵 초기화 ----
binMap          = false(nRows,nCols);
binMap(:,1)     = true;    % 왼쪽 벽
binMap(nRows,:) = true;    % 아래 벽
binMap(:,nCols) = true;    % 오른쪽 벽
% binMap(1,:)   = true;    % ← 위쪽도 막으려면 주석 해제

% ---- 장애물 삽입 ----
for k = 1:M
    r0 = obsRC(k,1);  c0 = obsRC(k,2);
    hv = obsSizeRC(k,2);   vv = obsSizeRC(k,1);  % h=가로, v=세로
    hr = floor(hv/2);      vr = floor(vv/2);     % 반경

    rIdx = max(r0-vr,1)      :  min(r0+vr, nRows);
    cIdx = max(c0-hr,1)      :  min(c0+hr, nCols);
    binMap(rIdx,cIdx) = true;
end
end