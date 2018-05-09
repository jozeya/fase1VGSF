#ifndef VEHICLE_H
#define VEHICLE_H
#pragma warning (disable:4786)
//ctime.h

#include "EntityMove.h"
#include <vector>
#include <iostream>
#include "Transformations.h"
#include "Wall2D.h"
#include "geometry.h"
//#include "GlobalFunctions.h"

using namespace std;

enum Deceleration{slow = 3, normal = 2, fast = 1};

class Vehicle : public EntityMove{
	private:
		double timeElapsed;
		vector<Vector2D> vecVehicleBuff; // buffer para la forma del vehiculo
		Vector2D SteeringForce;

		void initializeBuffer();
		//Vehicle (Vehicle &);
		//Vehicle& operator= (Vehicle &);

		//declaraciones wander
		double m_dWanderJitter;
		double m_dWanderRadius;
		double m_dWanderDistance;
		Vector2D m_vWanderTarget;
		double m_dWeightWander;

		//declaracion walls
		vector<Vector2D> m_Feelers;
		double m_dWallDetectionFeelerLength;

		double m_dWeightWallAvoidance;
		double m_dWeightFlee;
		double m_dWeightSeek;
		double m_dWeightArrive;
		double m_dWeightPursuit;
		double m_dWeightEvade;

		int conta;

		Deceleration m_Deceleration;

		Vehicle* m_pTargetAgent1;

		// Steering
		enum behavior_type{
						    none               = 0x00000,
						    seek               = 0x00002,
						    flee               = 0x00004,
						    arrive             = 0x00008,
						    wander             = 0x00010,
						    cohesion           = 0x00020,
						    separation         = 0x00040,
						    allignment         = 0x00080,
						    obstacle_avoidance = 0x00100,
						    wall_avoidance     = 0x00200,
						    follow_path        = 0x00400,
						    pursuit            = 0x00800,
						    evade              = 0x01000,
						    interpose          = 0x02000,
						    hide               = 0x04000,
						    flock              = 0x08000,
						    offset_pursuit     = 0x10000,
  		};

  		int m_iFlags;

	public:
		Vehicle(Vector2D position, double rotation, Vector2D vel, double _mass, double mForce, double mSpeed, double mTurnRate, double scale);

		~Vehicle();

		void Update(double);
		void Render();

		double getTimeElapsed(){
			return timeElapsed;
		}

		void setSteeringForce(Vector2D);

		vector<Vector2D> getVehicleBuffTrans();
		vector<Vector2D> getVehicleBuff();

		Vector2D Seek(Vector2D);
		Vector2D Flee(Vector2D);
		Vector2D Arrive(Vector2D,Deceleration);
		Vector2D Pursuit(const Vehicle*);
		Vector2D Wander();
		//void WanderOn();
		bool AccumulateForce(Vector2D &, Vector2D);
		double getWanderRadius(){ return m_dWanderRadius;}
		double getWanderDistance(){ return m_dWanderDistance;}
		Vector2D getWanderTarget(){return m_vWanderTarget;}
		void setWanderRadius(double r){m_dWanderRadius = r;}
		void CreateFeelers();
		Vector2D WallAvoidance(const vector<Wall2D>&);
		Vector2D CalculatePrioritized(const std::vector<Wall2D>&, Vector2D);
		Vector2D Evade(const Vehicle*);

		bool On(behavior_type bt){
			return (m_iFlags & bt) == bt;
		}

		void FleeOn(){
			m_iFlags |= flee;
		}

		void SeekOn(){
			m_iFlags |= seek;
		}

		void WanderOn(){
			m_iFlags |= wander;
		}

		void WallAvoidanceOn(){
			m_iFlags |= wall_avoidance;
		}
		
		void PursuitOn(Vehicle* v){
			m_iFlags |= pursuit; 
			m_pTargetAgent1 = v;
		}

		void EvadeOn(Vehicle* v){
			m_iFlags |= evade; 
			m_pTargetAgent1 = v;
		}
};

#endif