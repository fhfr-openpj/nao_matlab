function [left,right] = computeFootPressure(fsv,step)
    persistent l r
    if isempty(l); l = zeros(1e5,1); r = zeros(1e5,1); end  % 충분히 큰 버퍼
    % 센서 보정식
    Lf = [ 1  1  1  1;   1 -1 -1  1]; %#ok<NASGU>
    % 왼발
    lTmp = zeros(1,4);
    lTmp(1) = fsv(1,3)/3.4 + 1.5*fsv(1,1) + 1.15*fsv(1,2);
    lTmp(2) = fsv(1,3)/3.4 + 1.5*fsv(1,1) - 1.15*fsv(1,2);
    lTmp(3) = fsv(1,3)/3.4 - 1.5*fsv(1,1) - 1.15*fsv(1,2);
    lTmp(4) = fsv(1,3)/3.4 - 1.5*fsv(1,1) + 1.15*fsv(1,2);
    % 오른발
    rTmp = zeros(1,4);
    rTmp(1) = fsv(2,3)/3.4 + 1.5*fsv(2,1) + 1.15*fsv(2,2);
    rTmp(2) = fsv(2,3)/3.4 + 1.5*fsv(2,1) - 1.15*fsv(2,2);
    rTmp(3) = fsv(2,3)/3.4 - 1.5*fsv(2,1) - 1.15*fsv(2,2);
    rTmp(4) = fsv(2,3)/3.4 - 1.5*fsv(2,1) + 1.15*fsv(2,2);

    l(step) = sum(min(25,max(0,lTmp)));
    r(step) = sum(min(25,max(0,rTmp)));
    left  = l; right = r;
end