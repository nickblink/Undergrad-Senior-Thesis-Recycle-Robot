function testsimulateSCARA

robot = ScaraInit();
n = 1000;
state = zeros(6,n);  %  position on top of velocity
stateDer = zeros(6,n);
T = 1;
state(:,1) = [0 0 0 0 0 0]';
dt = T/(n-1);

 I = computeMoments;  % 

    function b = f(X,u)

        th1 = X(1);  th2 = X(2); 
        th1d = X(3); th2d = X(4);

        H = [I(4)+2*I(5)*cos(th2), I(3)+I(5)*cos(th2);
            I(3)+I(5)*cos(th2), I(6)];
        
        h = [-2*I(5)*sin(th2)*th1d*th2d - I(5)*sin(th2)*th2d^2;
            I(15)*sin(th2)*th1d^2];
        b = [th1d; th2d; 0; H\(h - u(1:2)); 0];

    end  % robot dynamics

        for i = 1:n-1 % time                  

            u = [.1; 0; 0];
            k1 = f(state(:,i),u);
            k2 = f(state(:,i)+.5*dt*k1,u);                
            k3 = f(state(:,i)+.5*dt*k2,u);        
            k4 = f(state(:,i)+dt*k3,u);     
            state(:,i+1) = state(:,i) + (1/6)*dt*(k1+2*k2+2*k3+k4);       


        end   

        statePath = [state(1,:); state(3,:); state(5,:)]';
        stateVelocity = [state(2,:); state(4,:); state(6,:)]';  
        keyboard
        
        robot.handles = drawScara(statePath(1,:),robot);
        for i = 2:n
            angles = statePath(i,:);
            [~,robot_T] = ScaraFK(angles,robot);
            for j = 1:3
                set(robot.handles(j),'Matrix',robot_T{j});
            end
            drawnow;   
            pause(.01)
%             mov(i) = getframe; 
        end   
        
  
end



function I = computeMoments

% Initialize robot
robot = ScaraInit();

% constants
m1 = robot.m_1;  % point mass of arm1 (the actuator for link 2 probably)
m2 = robot.m_2;
m3 = robot.m_3;
mr1 = robot.m_r1; % mass of link1, evenly distributed part
mr2 = robot.m_r2;
mr3 = robot.m_r3;

mr1 = 0;
mr2 = 0;  % first, for simplicity, assume that the joints are evenly distributed masses.
mr3 = 0;

l1 = robot.l_1; % length of link1
l2 = robot.l_2;
l3 = robot.l_3;

g = robot.g; % gravity

M1 = m1 + mr1;
M2 = m2 + mr2;
M3 = m3 + mr3;

cm1 = (mr1*l1/2 + m1*l1)/M1; % center of mass of link 1
cm2 = (mr2*l2/2 + m2*l2)/M2;
cm3 = (mr3*l3/2 + m3*l3)/M3;


%th1subd = -pi/2;  % desired final joint angles       
th2subd = -pi/2;
Kv = 30;  % constants for impedence controllers
Kp = 130;

Iz1 = l1^3/12;
Iz2 = Iz1;

r1 = l1/2;
r2 = l2/2;

% Moments
I(1) = mr1/(3 * l1) * ((l1 - cm1)^3 + cm1^3) + m1*(l1 - cm1)^2;
I(2) = mr2/(3 * l2) * ((l2 - cm2)^3 + cm2^3) + m2*(l2 - cm2)^2;


Mt3 = M3;  % Mt3 is mass of 3rd link including gripper and load mass
Mr2 = 0;  % rotor mass of motor in joint 2
n1 = 0;  % reduction rate in joint 1
n2 = 0;
Ir1 = 0;  % moment of rotor inertia of motor 1
Ir2 = 0;
            
I(1) = I(1);
I(2) = I(2);
I(3) = I(2) + (M2+M3)*cm2^2;  %(M2+M3) is the mass of 2nd and 3rd link and gripper and load
I(4) = I(1) + I(3) + n1*Ir1 + (Mr2+M2+M3)*l1^2;  % Here, M2 from the .pdf is M2+M3, as above
I(5) = M2*l1*cm2;
I(6) = I(3) + n2^2*Ir2;
I(7) = M3*l3^3/12;
I(10) = Mt3*(l1^2+l2^2);
I(8) = M3*(l1^2+l2^2)+I(7)+I(10);
I(9) = M3*l2^2+I(7);
I(11) = 2*M3*l2^2+2*I(7)+I(10);
I(12) = M3*l1*l2;
I(13) = Mt3*l1*l2;
I(14) = I(4) + I(8);
I(15) = I(5) + I(13);
I(16) = I(6) + I(9) + .25*I(10);
I(17) = I(3) + I(11);
I(18) = I(5) + 2*(I(12)+I(13));
I(19) = 1;  % M3

end