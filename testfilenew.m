clear all

robot_abs_pos = [0,0];
goal_abs_pos = [0,15];
obstacle_abs_position = [0,8;2,6];
no_of_obstacles =2;
dt = 0.2;
dummy = [0,0];
n=1; 
lambda = 2;
A= [];
b= [];
Aeq=[];
beq=[];
lb = -1.0 * ones(2);
ub = 1.0 * ones(2);
u0 = [0,1];
v_obstacle = [0,-0.2;-0.2,-0.4];
v_robot = [0,0];
v_max = 0.5;
v_desired = ((goal_abs_pos-robot_abs_pos)/norm(goal_abs_pos-robot_abs_pos))*v_max;
r_robot = 0.5;
r_obstacle = 2;
r_goal = 0.5;
v_r = [0,0];

while n==1
    
    %if dot((obstacle_abs_position-robot_abs_pos),v_obstacle)<=0 %| dot((obstacle_abs_position-robot_abs_pos),v_obstacle)==0
        objective = @(u) (norm(v_desired-(v_robot+u)))^2 + lambda * (norm(u)^2);
        nonlinear =  @(u)testconstraintnew(u,v_robot,v_obstacle,robot_abs_pos,obstacle_abs_position,r_robot,r_obstacle,v_r,no_of_obstacles);
        u= fmincon(objective,u0,A,b,Aeq,beq,lb,ub,nonlinear);
        
        
        
        
        %obstacle_abs_position = obstacle_abs_position+[0,-0.2];
        v_r = v_robot+u;
        
        if norm(v_r)>0.5 && norm(v_r)~=0
            v_r = (v_r/norm(v_r))*0.5;
        end
        robot_abs_pos = robot_abs_pos+v_r*dt;
        %v_obstacle = [0,-0.2]/dt; 
        v_desired = ((goal_abs_pos-robot_abs_pos)/norm(goal_abs_pos-robot_abs_pos))*v_max;
        
        circ(robot_abs_pos(1),robot_abs_pos(2),r_robot)
        hold on
        
        circ(goal_abs_pos(1),goal_abs_pos(2),r_goal)
        hold on
        
        for i=1:no_of_obstacles
            obstacle_abs_position(i,:) = obstacle_abs_position(i,:) + v_obstacle(i,:)*dt;
            circ(obstacle_abs_position(i,1),obstacle_abs_position(i,2),r_obstacle)
            hold on
        end
        
        
        xlim([-20 20])
        ylim([-20 20])    
        drawnow;
        hold off
        if norm(goal_abs_pos-robot_abs_pos)<1
            n=0;
        end
        
        
    end
    
    %{
    if dot((obstacle_abs_position-robot_abs_pos),v_obstacle) > 0
        robot_abs_pos = robot_abs_pos+((goal_abs_pos-robot_abs_pos)/norm(goal_abs_pos-robot_abs_pos))*0.1;
        
        circ(robot_abs_pos(1),robot_abs_pos(2),r_robot)
        hold on
        
        circ(obstacle_abs_position(1),obstacle_abs_position(2),r_obstacle)
        hold on
        
        circ(goal_abs_pos(1),goal_abs_pos(2),r_goal)
        hold on
        
        xlim([-20 20])
        ylim([-20 20])    
        drawnow;
        hold off
        if norm(goal_abs_pos-robot_abs_pos)<1
            n=0;
        end
        
    end
    
end
%}

function circ(x,y,r)
 th = 0:pi/50:2*pi;
 xunit = r * cos(th) + x;
 yunit = r * sin(th) + y;
 plot(xunit, yunit);
 axis square;
end

%........................................................................
%{
circ(2,5,2)
xlim([-15 16])
ylim([-15 16]) 
hold on
%}
%.......................................................................