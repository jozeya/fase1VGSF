#ifndef ENTITYBASE_H
#define ENTITYBASE_H

#include "Vector2D.h"

class EntityBase{
	public:
		enum {default_entity_type = -1};

	private:
		int entityID;
		int entityType;
		bool tag;

		int NextValidID(){
			static int nextId = 0;
			return nextId++;
		}

	protected:
		Vector2D entityPos;
		Vector2D entityScale;

		double boundingRadius;

		EntityBase():entityID(NextValidID()), boundingRadius(0.0), entityPos(Vector2D()), entityScale(Vector2D(1.0, 1.0)), entityType(default_entity_type), tag(false){

		}

		EntityBase(int entity_Type):entityID(NextValidID()), boundingRadius(0.0), entityPos(Vector2D()), entityScale(Vector2D(1.0, 1.0)), entityType(entity_Type), tag(false){

		}

		EntityBase(int entity_Type, Vector2D pos, double r):entityID(NextValidID()), boundingRadius(r), entityPos(pos), entityScale(Vector2D(1.0, 1.0)), entityType(entity_Type), tag(false){

		}

	public:
		
		virtual ~EntityBase(){}
		virtual void Update(double time_elapsed){};
		virtual void Render(){};

		Vector2D getPos() const{
			return entityPos;
		}

		void setPos(Vector2D newPos){
			entityPos = newPos;
		}

		double getRadius() const{
			return boundingRadius;
		}

		void setRadius(double r){
			boundingRadius = r;
		}

		int getID() const{
			return entityID;
		}

		bool isTagged() const{
			return tag;
		}

		void Tag(){
			tag = true;
		}

		void unTag(){
			tag = false;
		}

		Vector2D getScale() const{
			return entityScale;
		}

		void setScale(Vector2D newScale){
			boundingRadius *= MaxOf(newScale.x, newScale.y) / MaxOf(entityScale.x, entityScale.y); 
			entityScale = newScale;
		}

		void setScale(double newScale){
			boundingRadius *= (newScale / MaxOf(entityScale.x, entityScale.y));
			entityScale = Vector2D(newScale, newScale);
		}

		int getType() const{
			return entityType;
		}

		void setType(int newType){
			entityType = newType;
		}

};

#endif