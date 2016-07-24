traj_A = timeseries([0:24,24:-1:5,5:-0.5:1,1:-0.1:0],[0:3:132,134:2:150,152:2:172])
traj_B= timeseries([0:0.005:5,4.995:-0.005:0],[0:0.01:20])
traj_C=timeseries(1/80*[0:0.1:20,20,20:-0.1:0].^2,[0:0.1:20,20.5,20.6:0.1:40.6])
traj_D=timeseries(1/1600*[0:0.1:20].^3,[0:0.1:20])

Kennlinie=[50 65 75 85 100;600 930 1150 1350 1520]
[p,S]=polyfit(Kennlinie(1,:),Kennlinie(2,:),2)
fit=Kennlinie(1,:).^2*p(1)+Kennlinie(1,:)*p(2)+ p(3);
 
 
 %fit=Kennlinie(1,:)*p(1)+ p(2);
 plot(Kennlinie(1,:),Kennlinie(2,:))
 grid on 
 hold 
 plot(Kennlinie(1,:),fit)
 sqrt(sum((fit-Kennlinie(2,:)).^2)/5)