function [range, dis_goal, dis_center] = AvoidObstaclesRangeSensor(pose, mapMatrix, maxRange, mapScale)
% This function may be removed in the future.
% Copyright The MathWorks Inc, 2023
persistent r
 if isempty(r)
    r = rangeSensor('HorizontalAngle', [-3*pi/8, 3*pi/8], 'HorizontalAngleResolution', pi/8, 'Range', [0 maxRange]);
 end


% ── 지도 & 좌표 ──────────────────────────────────────────
map = binaryOccupancyMap(mapMatrix, mapScale);
p = pose';

x    = p(:,1);
y    = p(:,2);

% The range sensor accepts a pose on the map which is within the map's
% limits. Thus, filter out positions outside the map, and override them
% with readings that are zero.
xcord=p(:,1);
ycord=p(:,2);
xmin=map.XWorldLimits(1);
ymin=map.YWorldLimits(1);
xmax=map.XWorldLimits(2);
ymax=map.YWorldLimits(2);
isxvalid=xcord>=xmin&xcord<=xmax;
isyvalid=ycord>=ymin&ycord<=ymax;

% ── 맵 내부 여부 ─────────────────────────────────────────
isInside=isxvalid&isyvalid;

% ── 레이저 거리 ─────────────────────────────────────────
range = zeros(r.NumReadings,size(p,1));
range(:,isInside) = r(p(isInside,:), map);

% ── 목표(윗변)까지 세로 거리 ─────────────────────────────
dis_goal = ymax - y; 
dis_center = xmax/2 - x;
end

