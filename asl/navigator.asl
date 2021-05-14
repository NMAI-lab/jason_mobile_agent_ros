// Demo program of Jason based navigation using A*

{ include("D:/Local Documents/ROS_Workspaces/RoombaWorkspaces/src/jason_mobile_agent_ros/asl/obstacleHandler.asl")}
{ include("D:/Local Documents/ROS_Workspaces/RoombaWorkspaces/src/jason_mobile_agent_ros/asl/batteryManager.asl") }

mission(mission).
+!mission(Goal,Parameters)
	:	Goal = navigate
	    & Parameters = [Destination]
	<-	+mission(Goal,Parameters);
		!navigate(Destination);
		-mission(Goal,Parameters).
		
navigation(navigate).

// Case where we are already at the destination
+!navigate(Destination)
	:	position(X,Y) 
		& locationName(Destination,[X,Y])
	<-	.broadcast(tell, navigate(arrived(Destination))).

// We don't have a route plan, get one and set the waypoints.
+!navigate(Destination)
	:	position(X,Y) & locationName(Current,[X,Y])
	<-	.broadcast(tell, navigate(gettingRoute(Destination)));
		.broadcast(tell, navigate(current(Current)));
		?a_star(Current,Destination,Solution,Cost);
		.broadcast(tell, navigate(route(Solution,Cost), Destination));
		for (.member( op(_,NextPosition), Solution)) {
			!waypoint(NextPosition);
		}
		!navigate(Destination).
		
// Default plan
+!navigate(Destination)
	<-	.broadcast(tell, navigate(default,Destination)).

{ include("D:/Local Documents/ROS_Workspaces/RoombaWorkspaces/src/jason_mobile_agent_ros/asl/movement.asl") }

// Map of locations that the agent can visit.
{ include("D:/Local Documents/ROS_Workspaces/RoombaWorkspaces/src/jason_mobile_agent_ros/asl/map.asl") }

/* The following two rules are domain dependent and have to be redefined accordingly */

// sucessor definition: suc(CurrentState,NewState,Cost,Operation)
suc(Current,Next,1,up) :- ([X2,Y2] = [X1,Y1-1]) & possible(Current,Next) & nameMatch(Current,[X1,Y1],Next,[X2,Y2]).
suc(Current,Next,1,down) :- ([X2,Y2] = [X1,Y1+1]) & possible(Current,Next) & nameMatch(Current,[X1,Y1],Next,[X2,Y2]).
suc(Current,Next,1,left) :- ([X2,Y2] = [X1-1,Y1]) & possible(Current,Next) & nameMatch(Current,[X1,Y1],Next,[X2,Y2]).
suc(Current,Next,1,right) :- ([X2,Y2] = [X1+1,Y1]) & possible(Current,Next) & nameMatch(Current,[X1,Y1],Next,[X2,Y2]).

nameMatch(Current,CurrentPosition,Next,NextPosition) :- locationName(Current,CurrentPosition) &
														  locationName(Next,NextPosition).

// heutistic definition: h(CurrentState,Goal,H)
h(Current,Goal,H) :- H = math.sqrt( ((X2-X1) * (X2-X1)) + ((Y2-Y1) * (Y2-Y1)) ) &
						 nameMatch(Current,[X1,Y1],Goal,[X2,Y2]).

{ include("D:/Local Documents/ROS_Workspaces/RoombaWorkspaces/src/jason_mobile_agent_ros/asl/a_star.asl") }

