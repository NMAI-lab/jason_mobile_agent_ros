
safety(pedestrian).
@pedestrianAvoidance [atomic]
+pedestrian(_)
	<-	.broadcast(tell, pedestrian(honk(horn)));
		honk(horn).

map(obstacle).
+obstacle(Direction)
	:	position(X,Y) 
		& locationName(Current, [X,Y]) 
		& possible(Current,Next)
		& direction(Current,Next,Direction)
		& mission(Goal,Parameters)
	<-	-possible(Current,Next);
		//setObstacle(Current,Next);
		//.broadcast(tell, obstacle(Direction));
		.drop_all_intentions;
		!mission(Goal,Parameters).

