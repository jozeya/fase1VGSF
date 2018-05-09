#ifndef	ENTITYMOVE_H
#define ENTITYMOVE_H

#include "EntityBase.h"
#include "C2DMatrix.h"
#include <cassert>

class EntityMove : public EntityBase{
	protected:
		Vector2D velocity; // velocidad 
		Vector2D heading; // direccion del vector
		Vector2D side; // perperdicular a heading
		double mass;
		double maxSpeed;
		double maxForce;
		double maxTurnRate;

	public:

		EntityMove(Vector2D pos, double r, Vector2D vel, double ms, Vector2D hg, double _mass, Vector2D scale, double turnR, double mf) : 
					EntityBase(0, pos, r), heading(hg), velocity(vel), mass(_mass), side(heading.Perp()), maxSpeed(ms), maxTurnRate(turnR), maxForce(mf){

			EntityBase::entityScale = scale;
		}

		virtual ~EntityMove(){}

		Vector2D getVelocity() const{
			return velocity;
		}

		void setVelocity(Vector2D &newVel){
			velocity = newVel;
		}

		double getMass() const{
			return mass;
		}

		Vector2D getSide() const{
			return side;
		}	

		double getMaxSpeed() const{
			return maxSpeed;
		}

		void setMaxSpeed(double mS){
			maxSpeed = mS;
		}

		double getMaxForce() const{
			return maxForce;
		}

		void setMaxForce(double mF){
			maxForce = mF;
		}

		bool isSpeedMaxedOut() const{
			return ( maxSpeed * maxSpeed >= velocity.LengthSq() );
		}

		double getSpeed() const{
			return velocity.Length();
		}

		double getSpeedSq() const{
			return velocity.LengthSq();
		}

		Vector2D getHeading() const{
			return heading;
		}

		double getMaxTurnRate() const{
			return maxTurnRate;
		}

		void setMaxTurnRate(double val){
			maxTurnRate = val;
		}

		void setHeading(Vector2D nH);

		bool rotateHeadingToFacePosition(Vector2D target);

};

inline void EntityMove::setHeading(Vector2D nH){
	assert( (nH.LengthSq() - 1.0) < 0.00001 );
	heading = nH;
	side = nH.Perp();
}

inline bool EntityMove::rotateHeadingToFacePosition(Vector2D target){
	Vector2D toTarget = Vec2DNormalize(target - entityPos);
	double angle = acos(heading.Dot(toTarget));

	if (angle < 0.00001)
		return true;

	if (angle > maxTurnRate)
		angle = maxTurnRate;

	C2DMatrix rotationMatrix;

	rotationMatrix.Rotate(angle * heading.Sign(toTarget));
	rotationMatrix.TransformVector2Ds(heading);
	rotationMatrix.TransformVector2Ds(velocity);

	side = heading.Perp();

	return false;
}

#endif