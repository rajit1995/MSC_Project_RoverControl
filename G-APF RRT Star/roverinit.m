function Rover= roverinit(RRTState)
Rover.PointA = RRTState.PointA;
Rover.PointB = RRTState.PointB;
Rover.Length = 0.35;
Rover.Width = 0.2488;
Rover.n=2;
% Rover.LoSRadius = Rover.n*Rover.Length;
Rover.Radius = 0.5*norm([Rover.Length Rover.Width]);
% Rover.LoSRadius = Rover.n*(Rover.Length+Rover.Radius);
Rover.LoSRadius = Rover.n*(Rover.Length);
Rover.RotationTheta = pi/2;
Rover.Dimensions = RRTState.Dimensions;
Rover.Obstacles = RRTState.Obstacles;
Rover.StepSize =RRTState.StepSize;
Rover.Kpu = 15;
Rover.Kiu = 30;
Rover.kdh = 0.05;
Rover.Kph = 20;
Rover.Kih = 0.1;
Rover.kdu = 0.01;
Rover.dt = 0.001;
% Rover.dt = 0.01;
Rover.obstactalert = 0;
Rover.e_u_1 = [0 0];
Rover.u_sur_1 =[0 0];
Rover.waypoints = RRTState.finalpathvertices(:,1:2);
% Rover.pos_curr = Rover.waypoints(1,1:2);
Rover.pos_curr = Rover.waypoints(1,1:2);
Rover.pos_des =[0 0];
Rover.pos_des_1 =[0 0];
Rover.prev_wayPoint = Rover.waypoints(1,1:2);
Rover.Travel = [Rover.pos_curr];
Rover.counter = 1;
Rover.RadiusAcc = 0.4;
Rover.theta = 10;
Rover.theta_rad = Rover.theta*pi/180;
Rover.wpacc_ind = 0;
Rover.obst =[];
for j=1 : size(Rover.waypoints,1)
    ind =0;
            for i = 1:RRTState.Obstacles.Number
            [d,~,~] = p_poly_dist(Rover.waypoints(j,1),Rover.waypoints(j,2), Rover.Obstacles.X(i,:), Rover.Obstacles.Y(i,:)); 
            if d < 2*Rover.Radius 
                ind = ind +1;

            end

            end
              Rover.waypoints(j,3) = ind;
end
end