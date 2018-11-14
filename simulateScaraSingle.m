function [  ] = simulateScaraSingle(  )


close all;

pit = load('ScaraOwnMinPathSingle.mat');
n = pit.n;
control = pit.control;
T = pit.T;

dt = T/(n-1);
startVel = 0;
startPos = 0; 
startAccel = 0;  % startAccel

state = zeros(2,n);  %  position on top of velocity
stateDer = zeros(2,n);
state(:,1) = [startPos; startVel];
stateDer(:,1) = [startVel; startAccel];

function b = f(X,u)

        th1 = X(1);  
        th1d = X(2);
        b = [th1d; (1/3)*(u-4.405*cos(th1))];

    end  % robot dynamics

for i = 1:n-1 % time  
     
   u = control(i,:)';   % current control, row vector.
   stateDer(:,i) = f(state(:,i),u);
   state(:,i+1) = state(:,i) + dt*(stateDer(:,i)); %+ stateDer(:,i+1));                
    
end

statePath = state(1,:);
stateVelocity = state(2,:);

save('SinglePathCalc','statePath','stateVelocity');

%  X = pit.statePath';  %Uncomment if wanting to test the path that was generated

keyboard

visualizenRmanipulator

end


