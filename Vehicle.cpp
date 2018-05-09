#include "Vehicle.h"

Vehicle::Vehicle(Vector2D position, double rotation, Vector2D vel, double _mass, double mForce, double mSpeed, double mTurnRate, double scale) :
					EntityMove( position, scale, vel, mSpeed, Vector2D(sin(rotation), -cos(rotation)), _mass, Vector2D(scale, scale), mTurnRate, mForce),
					timeElapsed(0.0), m_Feelers(3){

	initializeBuffer();

	//wander
	m_dWanderJitter = 80.0;
	m_dWanderRadius = 1.2;
	m_dWanderDistance = 2.0;
	m_dWeightWander = 1.0;


	//walls 
	m_dWallDetectionFeelerLength = 40.0;

	//other steering
	m_dWeightWallAvoidance = 10.0;
	m_dWeightFlee = 1.0;
	m_dWeightSeek = 1.0;
	m_dWeightArrive = 1.0;
	m_dWeightPursuit = 1.0;
	m_dWeightEvade = 0.01;

	m_Deceleration = normal;

	conta  = 0;
}

Vehicle::~Vehicle(){

}

void Vehicle::Update(double time_elapsed){
	timeElapsed = time_elapsed;
	Vector2D oldPos = getPos();

	//SteeringForce.Zero();

	//SteeringForce = CalculatePrioritized(walls, obj);


    Vector2D acceleration = SteeringForce / mass;
	
	velocity += acceleration * time_elapsed; 
	
	//make sure vehicle does not exceed maximum velocity
  	velocity.Truncate(maxSpeed);
	
	entityPos += velocity * time_elapsed;

	//update the heading if the vehicle has a non zero velocity

	//setHeading(Vec2DNormalize(velocity));
	if (velocity.LengthSq() > 0.00000001){    
		heading = Vec2DNormalize(velocity);	
		
		side = heading.Perp();
    }

    WrapAround(entityPos, 640, 480);
}

void Vehicle::Render(){
	static std::vector<Vector2D>  m_vecVehicleVBTrans;

	m_vecVehicleVBTrans = WorldTransform( vecVehicleBuff, getPos(), getHeading(), getSide(), getScale() );

	//ClosedShape(m_vecVehicleVBTrans);
}

void Vehicle::initializeBuffer(){
	const int numVehicleVerts = 3;

//	Vector2D vehicle[numVehicleVerts] = { Vector2D(-1.0f, 0.6f), Vector2D(1.0f, 0.0f), Vector2D(-1.0f, -0.6f) };
	int x = 1;
	Vector2D vehicle[numVehicleVerts] = { Vector2D(-1.0f*x, 0.6f*x), Vector2D(1.0f*x, 0.0f*x), Vector2D(-1.0f*x, -0.6f*x) };

	for (int i = 0; i < numVehicleVerts; ++i){
		vecVehicleBuff.push_back(vehicle[i]);
	}
}

vector<Vector2D> Vehicle::getVehicleBuffTrans(){
	static std::vector<Vector2D>  m_vecVehicleVBTrans;

	m_vecVehicleVBTrans = WorldTransform( vecVehicleBuff, getPos(), getHeading(), getSide(), getScale() );

	Vector2D tmpPos = getPos();
	Vector2D tmpHeading = getHeading();
	Vector2D tmpSide = getSide();
	Vector2D tmpScale = getScale();

	//cout << "Pos = " << tmpPos.x << ", " << tmpPos.y << endl;
	//cout << "Heading = " << tmpHeading.x << ", " << tmpHeading.y << endl;
	//cout << "Side = " << tmpSide.x << ", " << tmpSide.y << endl;
	//cout << "Scale = " << tmpScale.x << ", " << tmpScale.y << endl;


	//ClosedShape(m_vecVehicleVBTrans); 
	return m_vecVehicleVBTrans;
}

vector<Vector2D> Vehicle::getVehicleBuff(){
	return vecVehicleBuff;
}


Vector2D Vehicle::Seek(Vector2D TargetPos){

	Vector2D DesiredVelocity = Vec2DNormalize(TargetPos - getPos()) * getMaxSpeed();

	SteeringForce = (DesiredVelocity - getVelocity());
	return (DesiredVelocity - getVelocity());
}

Vector2D Vehicle::Flee(Vector2D TargetPos){

	/*double PanicDistanceSq = 100.0 * 100.0;

	if (Vec2DDistanceSq(getPos(), TargetPos) > PanicDistanceSq){
		//SteeringForce = Vector2D(0,0);
		return Vector2D(0,0);
	}*/

	Vector2D DesiredVelocity = Vec2DNormalize( getPos() - TargetPos) * getMaxSpeed();

	SteeringForce = (DesiredVelocity - getVelocity());

	return (DesiredVelocity - getVelocity());
}

void Vehicle::setSteeringForce(Vector2D TargetPos){
	//SteeringForce = Vec2DNormalize(TargetPos - getPos()) * getMaxSpeed();
	SteeringForce = TargetPos;
}

Vector2D Vehicle::Arrive(Vector2D TargetPos, Deceleration deceleration){
	Vector2D ToTarget = TargetPos - getPos();
	//calculate the distance to the target position
	double dist = ToTarget.Length();

	if (dist > 0){
		//because Deceleration is enumerated as an int, this value is required
		//to provide fine tweaking of the deceleration.
		const double DecelerationTweaker = 0.3;
		//calculate the speed required to reach the target given the desired
		//deceleration
		double speed = dist / ((double)deceleration * DecelerationTweaker);
		//make sure the velocity does not exceed the max
		speed = min(speed, getMaxSpeed());
		//from here proceed just like Seek except we don't need to normalize
		//the ToTarget vector because we have already gone to the trouble
		//of calculating its length: dist.
		Vector2D DesiredVelocity = ToTarget * speed / dist;
		SteeringForce = (DesiredVelocity - getVelocity());
		return (DesiredVelocity - getVelocity());
	}
	SteeringForce = Vector2D(0,0);
	return Vector2D(0,0);
}

Vector2D Vehicle::Pursuit(const Vehicle* evader){
	//for the evader's current position.
	Vector2D ToEvader = evader -> getPos() - getPos();

	double RelativeHeading = getHeading().Dot(evader -> getHeading());

	if ((ToEvader.Dot(getHeading()) > 0) && (RelativeHeading < -0.95)){ //acos(0.95)=18 degs
		return Seek(evader->getPos());
	}
	
	//Not considered ahead so we predict where the evader will be.
	//the look-ahead time is proportional to the distance between the evader
	//and the pursuer; and is inversely proportional to the sum of the
	//agents' velocities
	double LookAheadTime = ToEvader.Length() / (getMaxSpeed() + evader -> getSpeed());
	//now seek to the predicted future position of the evader
	return Seek(evader->getPos() + evader->getVelocity() * LookAheadTime);
}

Vector2D Vehicle::Wander(){
	//this behavior is dependent on the update rate, so this line must
 	//be included when using time independent framerate.
  	double JitterThisTimeSlice = m_dWanderJitter * 0.004;

	//first, add a small random vector to the target's position
  	m_vWanderTarget += Vector2D(RandomClamped() * JitterThisTimeSlice, RandomClamped() * JitterThisTimeSlice);
	
	//reproject this new vector back on to a unit circle
	m_vWanderTarget.Normalize();

  	//increase the length of the vector to the same as the radius
  	//of the wander circle
  	m_vWanderTarget *= m_dWanderRadius;

  	//move the target into a position WanderDist in front of the agent
  	Vector2D target = m_vWanderTarget + Vector2D(m_dWanderDistance, 0);

  	//project the target into world space
  	Vector2D Target = PointToWorldSpace(target, getHeading(), getSide(), getPos());

	//and steer towards it
  	return Target - getPos(); 


}

/*void Vehicle::WanderOn(){

	Vector2D force;
	force = Wander() * m_dWeightWander;

    if (!AccumulateForce(SteeringForce, force)) {
    	return ;
    }
    SteeringForce += Wander();
	//Wander();
}*/

bool Vehicle::AccumulateForce(Vector2D &RunningTot, Vector2D ForceToAdd) {
  
  //calculate how much steering force the vehicle has used so far
  double MagnitudeSoFar = RunningTot.Length();

  //calculate how much steering force remains to be used by this vehicle
  double MagnitudeRemaining = getMaxForce() - MagnitudeSoFar;

  //return false if there is no more force left to use
  if (MagnitudeRemaining <= 0.0) return false;

  //calculate the magnitude of the force we want to add
  double MagnitudeToAdd = ForceToAdd.Length();
  
  //if the magnitude of the sum of ForceToAdd and the running total
  //does not exceed the maximum force available to this vehicle, just
  //add together. Otherwise add as much of the ForceToAdd vector is
  //possible without going over the max.
  if (MagnitudeToAdd < MagnitudeRemaining){
    RunningTot += ForceToAdd;
  }
  else{
    //add it to the steering force
    RunningTot += (Vec2DNormalize(ForceToAdd) * MagnitudeRemaining); 
  }

  return true;
}

Vector2D Vehicle::WallAvoidance(const std::vector<Wall2D>& walls){
	//the feelers are contained in a std::vector, m_Feelers
	CreateFeelers();

	double DistToThisIP = 0.0;
	double DistToClosestIP = MaxDouble;

	//this will hold an index into the vector of walls
	int ClosestWall = -1;

	Vector2D mSteeringForce, 
			 point,//used for storing temporary info
			 ClosestPoint; //holds the closest intersection point
	
	//examine each feeler in turn
	for (int flr=0; flr < m_Feelers.size(); ++flr){
		//run through each wall checking for any intersection points
		for (int w=0; w < walls.size(); ++w){
			if (LineIntersection2D(getPos(), m_Feelers[flr], walls[w].From(), walls[w].To(), DistToThisIP, point)) {
				//is this the closest found so far? If so keep a record
				if (DistToThisIP < DistToClosestIP) {
					DistToClosestIP = DistToThisIP;
					ClosestWall = w;
					ClosestPoint = point;
				}
			}
		}//next wall
		//if an intersection point has been detected, calculate a force
		//that will direct the agent away
		if (ClosestWall >= 0) {
			//calculate by what distance the projected position of the agent
			//will overshoot the wall
			Vector2D OverShoot = m_Feelers[flr] - ClosestPoint;
			//create a force in the direction of the wall normal, with a
			//magnitude of the overshoot
			mSteeringForce = walls[ClosestWall].Normal() * OverShoot.Length();
		}
	}//next feeler

	return mSteeringForce;
}

void Vehicle::CreateFeelers(){
  //feeler pointing straight in front
  m_Feelers[0] = getPos() + m_dWallDetectionFeelerLength * getHeading();

  //feeler to left
  Vector2D temp = getHeading();
  Vec2DRotateAroundOrigin(temp, HalfPi * 3.5f);
  m_Feelers[1] = getPos() + m_dWallDetectionFeelerLength/2.0f * temp;

  //feeler to right
  temp = getHeading();
  Vec2DRotateAroundOrigin(temp, HalfPi * 0.5f);
  m_Feelers[2] = getPos() + m_dWallDetectionFeelerLength/2.0f * temp;
}

Vector2D Vehicle::CalculatePrioritized(const std::vector<Wall2D>& walls, Vector2D obj){       
	Vector2D force;
	//SteeringForce.Zero();

	if (On(wall_avoidance)){
		
		force = WallAvoidance(walls) * m_dWeightWallAvoidance;

		if (!AccumulateForce(SteeringForce, force)) {
			cout << ++conta <<" Aun no se acumulo fuerza" << endl;
			return SteeringForce;
		}
		cout << ++conta<<" Fuerza acumulada -----" << endl;
	}

	if (On(flee)){
		force = Flee(obj) * m_dWeightFlee; 
		if (!AccumulateForce(SteeringForce, force)) 
			return SteeringForce;
	}


	if (On(seek)){
		force = Seek(obj) * m_dWeightSeek;

		if (!AccumulateForce(SteeringForce, force)) 
			return SteeringForce;
	}


	if (On(arrive)){
		force = Arrive(obj, m_Deceleration) * m_dWeightArrive;

		if (!AccumulateForce(SteeringForce, force)) 
			return SteeringForce;
	}

	if (On(wander)){
		force = Wander() * m_dWeightWander;

		if (!AccumulateForce(SteeringForce, force)) 
			return SteeringForce;
	}

	if (On(pursuit)){
		assert(m_pTargetAgent1 && "pursuit target not assigned");

		force = Pursuit(m_pTargetAgent1) * m_dWeightPursuit;

		if (!AccumulateForce(SteeringForce, force)) 
			return SteeringForce;
	}

	if (On(evade)){
    	assert(m_pTargetAgent1 && "Evade target not assigned");
    
    	force = Evade(m_pTargetAgent1) * m_dWeightEvade;

    	if (!AccumulateForce(SteeringForce, force)) 
    		return SteeringForce;
    }

	return SteeringForce;
}

Vector2D Vehicle::Evade(const Vehicle* pursuer){

	/* Not necessary to include the check for facing direction this time */
	Vector2D ToPursuer = pursuer -> getPos() - getPos();

	//uncomment the following two lines to have Evade only consider pursuers 
	//within a 'threat range'
	const double ThreatRange = 100.0;

	if (ToPursuer.LengthSq() > ThreatRange * ThreatRange) 
		return Vector2D();

	//the lookahead time is propotional to the distance between the pursuer
	//and the pursuer; and is inversely proportional to the sum of the
	//agents' velocities
	double LookAheadTime = ToPursuer.Length() / (getMaxSpeed() + pursuer->getSpeed());

	//now flee away from predicted future position of the pursuer
	return Flee(pursuer->getPos() + pursuer->getVelocity() * LookAheadTime);
}