function Rover = obstacledetect(Rover)
  Rover.pos_curr1 = Rover.pos_curr + Rover.disp;
 for j=1:36
       Rover.RovCircle(j,:) = [(Rover.pos_curr1(1) + (Rover.Radius+0.1)*cos(j*Rover.theta_rad)) (Rover.pos_curr1(2) + (Rover.Radius+0.1)*sin(j*Rover.theta_rad))];      
  end
    poly1 = polyshape([Rover.RovCircle(:,1)],[Rover.RovCircle(:,2)]);
    
    for j= 1:Rover.Obstacles.Number
        poly2 = polyshape([Rover.Obstacles.X(j,:)],[Rover.Obstacles.Y(j,:)]);
        polyout = intersect(poly1,poly2);
        Rover.polyind(j,:) =  polyout.NumRegions;
        if polyout.NumRegions > 0
            Rover.obs_center = Rover.Obstacles.Centers(j,:);
        end
    end
    Rover.poly_ind = sum(Rover.polyind);
    if Rover.polyind == 0
        Rover.pos_curr = Rover.pos_curr1;
        Rover.obstactalert = 0;
    else
       % new_head= acos(sum(Rover.pos_curr.*Rover.next_wayPoint)/(norm(Rover.pos_curr)*norm(Rover.next_wayPoint)));
       new_head1 = atan(Rover.pos_curr(2)/Rover.pos_curr(1));
       new_head2 = atan(Rover.next_wayPoint(2)/Rover.next_wayPoint(1));
       new_head = new_head2 - new_head1;
       Rover.obstactalert = 1;
       if new_head > 0
           new_head_angle = Rover.RotationTheta;
       elseif new_head < 0
           new_head_angle =  -1* Rover.RotationTheta;
       else 
            new_head_u = Rover.pos_curr - Rover.obs_center/norm(Rover.pos_curr - Rover.obs_center);
            new_head3 = atan(new_head_u(2)/new_head_u(1));
            new_head2_u = Rover.u_sur/norm(Rover.u_sur);
            new_head2 = atan(new_head2_u(2)/new_head2_u(1));
            new_head_e  = new_head3 - new_head2;
            

           if new_head_e > 0
            new_head_angle =  Rover.RotationTheta; 
           else 
            new_head_angle = -1* Rover.RotationTheta;
           end
       end
       % Rover.obst =  [Rover.obst;Rover.poly_ind];
        % pos_curr = Rover.pos_curr;
        Dcos = [cos(new_head_angle) -sin(new_head_angle);sin(new_head_angle) cos(new_head_angle)] ;
        Rover.u_sur = transpose(Dcos*transpose(Rover.u_sur));
        Rover.disp = Rover.u_sur*Rover.dt;
        
        Rover.pos_curr = Rover.pos_curr + Rover.disp;
        % Rover.Travel = [Rover.Travel;Rover.pos_curr];
        Rover.obstactalert = 1;
        % visualrover(Rover);
        visualrover(Rover);
        Rover = obstacledetect(Rover);
        % Rover.pos_curr1 = Rover.pos_curr + Rover.disp;
        

    end
    Rover.Travel = [Rover.Travel;Rover.pos_curr];
    
end