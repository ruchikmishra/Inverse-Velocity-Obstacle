

function[c,ceq]= testconstraintnew(u,v_robot,v_obstacle,robot_abs_pos,obstacle_abs_position,r_robot,r_obstacle,v_r,no_of_obstacles)
c=zeros(1,2);

for i=1:no_of_obstacles
    %if norm(obstacle_abs_position(i,:)-robot_abs_pos)<=5
    if dot((obstacle_abs_position(i,:)-robot_abs_pos),(v_obstacle(i,:)-v_r))<=0 && norm(obstacle_abs_position(i,:)-robot_abs_pos)<=5
        %c(i,:)= (dot(transpose(obstacle_abs_position-robot_abs_pos),(v_obstacle-(v_robot+u)))/norm(v_obstacle-(v_robot+u)))^2 + (r_robot+r_obstacle)^2 - (norm(obstacle_abs_position-robot_abs_pos))^2;
        c(i)= (dot(transpose(obstacle_abs_position(i,:)-robot_abs_pos),(v_obstacle(i,:)-(v_robot+u)))/norm(v_obstacle(i,:)-(v_robot+u)))^2 + (r_robot+r_obstacle)^2 - (norm(obstacle_abs_position(i,:)-robot_abs_pos))^2;
        ceq=[];
    else
        c(i)=0;
        ceq=[];
    end
    end
end
