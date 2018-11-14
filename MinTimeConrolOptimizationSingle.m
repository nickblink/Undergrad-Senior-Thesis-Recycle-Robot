function MinTimeConrolOptimizationSingle  

% takes advantage of scopes in Matlab in order to reduce 
% number of inputs required for auxillary functions.
                                   
clc;

n = 40;  % number of time steps
dtau = 1/(n-1);
% pit = load('ScaraOwnMinPathSingle.mat');
% X0 = pit.X0;

M = 10;

SZ = 3*n+1;
X0 = zeros(SZ,1);

startState = zeros(2,1);
finishState = [2 0]';


    function b = f(X,u)

        th1 = X(1);  
        th1d = X(2);
        b = [th1d; (1/3)*(u-4.405*cos(th1))];

    end  % robot dynamics

    function b = F(X)

        b = zeros((n+1)*2,1);    

        for i = 1:n-1
            b(Elem(i,n)) = (X(Elem(i+1,n)) - X(Elem(i,n)))/dtau - ... 
                X(end)*f(X(Elem(i,n)),X(Elem(n+i,n)));

        end
        
        %b(Elem(n,n)) = b(Elem(n-1,n));
        b(end-3:end-2) = startState - X(Elem(1,n));
        b(end-1:end) = finishState - X(Elem(n,n));

    end    % Newton Optimization Equality Constraints

    while(sum(abs(F(X0))) > .1)   % could save an X0 that works, save a few seconds
        X0 = rand(SZ,1);
        X0 = fsolve(@F,X0);
    end

    lb = [-inf*ones(2*n,1); -M*ones(n,1); 0];
    ub = [inf*ones(2*n,1); M*ones(n,1); inf];
    opt = optimset('Algorithm','sqp');
    opt.MaxFunEvals = 10000;
    opt.TolFun = .3; %*ones(SZ,n+2)';   % maybe increasing the tolerance would help?, size of b = F(X)

    function [Cout,Ceq] = Ftemp(X)  % Wrapper on Newton Optimization
        Ceq = F(X);
        Cout = [];
    end

    flag = 0;
Sca
    %while flag ~= 1
    [X,~,flag] = fmincon(@Objective,X0,[],[],[],[],lb,ub,@Ftemp,opt);
    %end

    for i = 1
      statePath(:,i) = X(1:2:2*n);
      stateVelocity(:,i) = X(2:2:2*n);
      control(:,i) = X(2*n+1:end-1);
      T = X(end);
    end

    
    
    save('ScaraOwnMinPathSingle','statePath','n','control','startState','finishState','T','X0','stateVelocity');


end


function b = Objective(x)

    b = x(end); 

end

function ind = Elem(i,n)

    if i <= n
        ind = (2*(i-1) + 1):2*i; % for all state variables
    elseif i <= 2*n
        ind = 2*n + (i-n-1) + 1;
    end

    if i == 2*n+1
        ind = 2*n+1;
    end

end  %% needs to be sped up

   