function [xyHits,binMap] = lidar2OccMap(rangeMat,lidar_param,nl,nc)
    hRes   = lidar_param.hRes;
    fov    = lidar_param.fov;
    nLayer = lidar_param.nLayer;
    maxR   = lidar_param.maxR;

    rValues  = rangeMat(ceil(nLayer/2),:);
    thetaVec = linspace(fov/2,-fov/2,hRes);
    xEnd     = -rValues .* sin(thetaVec);
    yEnd     =  rValues .* cos(thetaVec);

    tol = 6;
    isHit = (rValues < maxR-tol) & ~isinf(rValues) & ~isnan(rValues);
    xyHits = [xEnd(isHit).',yEnd(isHit).'];

    if isempty(xyHits)
        binMap = createBinaryMap(nl,nc,[],[3 4]);
        return
    end

    epsDB = 0.15; minPts = 4;
    clabel = dbscan(xyHits,epsDB,minPts);
    valid  = clabel > 0;
    xyC    = xyHits(valid,:);

    coords = xyC*10; coords(:,1) = coords(:,1)+10;
    obsRow = nl - ceil(coords(:,2)) + 1;
    obsCol = ceil(coords(:,1));
    obsRC  = [obsRow obsCol];
    binMap = createBinaryMap(nl,nc,obsRC,[3 4]);
end