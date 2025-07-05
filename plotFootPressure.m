function plotFootPressure(left,right,steps,dt)
if steps <= 100
    time = (1:steps)*dt/1000;
    plot(time,left(1:steps),'b',time,right(1:steps),'r');
else
    time = (steps-100:steps)*dt/1000;
    plot(time,left(steps-100:steps),'b',time,right(steps-100:steps),'r');
end
title('Left/right foot pressure'); xlabel('time [s]'); ylabel('Newtons [N]');
end