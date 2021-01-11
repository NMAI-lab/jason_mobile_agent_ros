// Demo program of Jason based navigation using A*

!navigate(d).

/*
// Benchmark version
+path(Path)
	:	startTime(StartTime)
	<-	// Print the results
		.print("solution A* =", Path, " in ", (system.time - StartTime), " ms.");
		+done.

+!navigate(Destination)
	: 	(not done)
	<-	+startTime(system.time);		// Get initial time stamp, for benchmarking performance
		getPath(a,Destination);
		!navigate(Destination).

+!navigate(_) <- .print("Done").
*/

// Case where we are already at the destination
+!navigate(Destination)
	:	position(X,Y) & locationName(Destination,[X,Y])
	<-	.print("Made it to the destination!");
		-destinaton(Destination);
		-route(Path).

// We are not at the destination, set the waypoints.
+!navigate(Destination)
	:	position(X,Y) 
		& locationName(Current,[X,Y])
	<-	savi_ros_java.savi_ros_bdi.navigation.getPath(Current,Destination,Path);
		for (.member(NextPosition, Path)) {
			!waypoint(NextPosition);
		}
		!navigate(Destination).	

+!navigate(Destination)
	<-	!navigate(Destination).
	

// Move through the map, if possible.
+!waypoint(NextPosition)
	:	position(X,Y) & locationName(Current,[X,Y])
		& possible(Current,NextPosition)
		& direction(Current,NextPosition,Direction)
		& map(Direction)
		& (not obstacle(Direction))
	<-	move(Direction).
	
// Move through the map, if possible.
//+!waypoint(NextPosition)
//	:	isDirection(Direction) &
//		map(Direction) &
//		obstacle(Direction)
//	<-	!updateMap(Direction, Next).

// Deal with case where Direction is not a valid way to go.
+!waypoint(Next) <- .print("Waypoint default").


// Revisit map update later.
/*
+!updateMap(Direction, NextName)
	:	position(X,Y) &
		locationName(PositionName, [X,Y]) &
		possible(PositionName,NextName) &
		destination(Destination)
	<-	-possible(PositionName,NextName)
		.print("Did map update ", Direction, " ", NextName);
		.drop_all_intentions;
		!navigate(Destination).
	
+!updateMap(Direction,NextName)
	<-	.print("Map update default ",Direction, " ", NextName);
		!updateMap(Direction,NextName).
	
*/

// Get the direction of the next movement
direction(Current,Next,up)
	:-	possible(Current,Next)
		& locationName(Current,[X,Y])
		& locationName(Next,[X,Y-1]).

// Get the direction of the next movement
direction(Current,Next,down)
	:-	possible(Current,Next)
		& locationName(Current,[X,Y])
		& locationName(Next,[X,Y+1]).
		
// Get the direction of the next movement
direction(Current,Next,left)
	:-	possible(Current,Next)
		& locationName(Current,[X,Y])
		& locationName(Next,[X-1,Y]).		
		
// Get the direction of the next movement
direction(Current,Next,right)
	:-	possible(Current,Next)
		& locationName(Current,[X,Y])
		& locationName(Next,[X+1,Y]).			

// Map of locations that the agent can visit.
{ include("D:/Local Documents/ROS_Workspaces/RoombaWorkspaces/src/jason_mobile_agent_ros/asl/map.asl") }

