function [figOut,axOut, lidarParam] = updateLidarPolar(rangeMat,lidar,figIn,axIn)
% LiDAR 극좌표 스캔을 한 창에서 지속적으로 업데이트
persistent lidarLines validPts           % 그래픽 핸들 캐시

% ── 0. Figure/axes 확보 ───────────────────────────────────────────────
figOut = figIn;  axOut = axIn;
if isempty(figOut) || ~isgraphics(figOut)
    figOut = figure('Name','Lidar Scan (polar)','NumberTitle','off');
    axOut  = axes('Parent',figOut);
    hold(axOut,'on'); axis(axOut,'equal'); grid(axOut,'on');
    xlabel(axOut,'X [m]'); ylabel(axOut,'Y [m]');
    lidarLines = gobjects(0); validPts = gobjects(1);
end

lidarParam = struct('hRes',0,'fov',0,'nLayer',0,'maxR',0, ...
                    'rValues',[],'thetaVec',[],'xEnd',[],'yEnd',[]);


% ── 1. rangeMat이 있을 때만 빔 계산 ─────────────────────────────────
if ~isempty(rangeMat)
    hRes   = wb_lidar_get_horizontal_resolution(lidar);
    nLayer = wb_lidar_get_number_of_layers(lidar);
    fov    = wb_lidar_get_fov(lidar);
    maxR   = wb_lidar_get_max_range(lidar);

    rValues  = rangeMat(ceil(nLayer/2),:);
    thetaVec = linspace( fov/2, -fov/2, hRes);
    xEnd = -rValues .* sin(thetaVec);
    yEnd =  rValues .* cos(thetaVec);

    lidarParam = struct('hRes',hRes,'fov',fov,'nLayer',nLayer,'maxR',maxR, ...
                        'rValues',rValues,'thetaVec',thetaVec, ...
                        'xEnd',xEnd,'yEnd',yEnd);

    needNew = isempty(lidarLines) || numel(lidarLines)~=hRes || ...
              ~all(isgraphics(lidarLines));
    if needNew
        delete(lidarLines(isgraphics(lidarLines)));      % 누적된 라인 제거
        lidarLines = gobjects(1,hRes);
        for k = 1:hRes
            lidarLines(k) = line(axOut,[0 xEnd(k)],[0 yEnd(k)], ...
                                  'Color',[0 0.6 1 0.6]);
        end
        t = linspace(0,2*pi,360);                        % 최대 사거리 원
        plot(axOut,maxR*cos(t),maxR*sin(t),'k:');
        title(axOut,'Lidar Polar Scan');
    else
        for k = 1:hRes
            set(lidarLines(k),'XData',[0 xEnd(k)],'YData',[0 yEnd(k)]);
        end
    end

    % 히트 포인트(빨간 점)
    isHit = rValues < (maxR - 6);
    if ~isempty(validPts) && isgraphics(validPts)
        set(validPts,'XData',xEnd(isHit),'YData',yEnd(isHit));
    else
        validPts = scatter(axOut,xEnd(isHit),yEnd(isHit), ...
                           15,'filled','r');
    end
    xlim(axOut,[-maxR maxR]); ylim(axOut,[-maxR maxR]);

else   % ── 2. rangeMat 비어 있음(워밍업) ──────────────────────────────
    cla(axOut); title(axOut,'Lidar Scan (warming up)');
    lidarLines = gobjects(0); validPts = gobjects(1);
end
end
