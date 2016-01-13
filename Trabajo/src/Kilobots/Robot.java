package Kilobots;

import sim.engine.*;
import sim.field.continuous.*;
import sim.util.*;

public class Robot implements Steppable
{
	double DESIRED_DISTANCE = 3.2;
	double orientation;
	int gradientValue=0;
	public boolean isStationary = true;
	public boolean isReference = false;
	public boolean isMoving = false;
	public boolean validGradient = false;
	
	public boolean inCollision = false;
	
	private double previousDistance = Double.MAX_VALUE;
	Bag neighbourhood;
	
	Double2D me;
	Continuous2D space;
	
	public void setMoving (boolean isFollowing) {isMoving = isFollowing;}
	public boolean getMoving () {return isMoving;}
	
	public boolean getIsReference() {return isReference;}
	public void setOrientation(double o) {orientation = o;}
	public double orientation2D () {return orientation;}
	public int getGradientValue () {return gradientValue;}
	public void setDesiredDistance(double d) {DESIRED_DISTANCE = d;}
	public double getDesiredDistance() { return DESIRED_DISTANCE;}
	
	//public int getNumNeigh() {return numNeig;}
	
	public void step(SimState state)
	{
		
		if(isReference) return;
		
		Swarm swarm = (Swarm) state;
		space = swarm.space;
		
		Double2D aux = space.getObjectLocation(this);
		if(me!=aux) validGradient = false;
		me = aux;
		neighbourhood = space.getNeighborsWithinDistance(me, 10);	
		
		if (isMoving)
			followEdge();
		else 
			calculateGradient();
		
		if (validMovement (me))
			inCollision = false;
		else
			inCollision = true;
		
		
	}
	
	private void moveForward ()
	{
		MutableDouble2D nextPosition = new MutableDouble2D();
		nextPosition.addIn(Math.cos(orientation)*0.1, Math.sin(orientation)*0.1);
		nextPosition.addIn(me);
		
		//if(validMovement(nextPosition))
			space.setObjectLocation(this, new Double2D(nextPosition));
	}
	
	private boolean validMovement (Double2D nextPosition)
	{	
		Bag neighbors = space.getNeighborsWithinDistance(nextPosition, 3.1);
		
		for (int i = 0; i < neighbors.size(); i++)
		{
			if (neighbors.get(i) == this ) 
				continue;
			if (Swarm.checkCollision( space.getObjectLocation(neighbors.get(i)) , me))
				return false;
		}
		return true;
	}
	
	/*private boolean validMovement (MutableDouble2D nextPosition)
	{
		return validMovement (new Double2D(nextPosition));
	}*/
	
	private void rotate(boolean direction)
	{
		if (direction)
		{
			orientation += 0.6;
			if (orientation >= 6.28319)
				orientation -= 6.28319;
		}
		else
			orientation -= 0.6;
			if (orientation < 0)
				orientation += 6.28319;
	}
	
	private void followEdge ()
	{
		double distance = Double.MAX_VALUE;
		double auxDistance;
		for (int i = 0; i<neighbourhood.size(); i++)
		{
			if (neighbourhood.get(i) == this || ((Robot)neighbourhood.get(i)).isMoving) 
				continue;
			auxDistance = Swarm.getDistance (space.getObjectLocation(neighbourhood.get(i)), me);
			
			if(auxDistance < distance)
				distance = auxDistance;
		}
		
		if (distance < DESIRED_DISTANCE)
		{
			if (previousDistance < distance)
				moveForward();
			else 
			{
				
				moveForward();
				rotate (false);
			}
		}
		else
			if (previousDistance > distance)
				moveForward();
			else 
			{
				
				moveForward();
				rotate (true);
			}
				
		previousDistance = distance;
		
	}
	
	private void calculateGradient()
	{
		//TODO probar a quitar current y cambiar el valor directamente
		Bag neighbors = space.getNeighborsExactlyWithinDistance(me, 4.5);
		int current = Integer.MAX_VALUE;
		int neighValue;
		for (int i = 0; i < neighbors.size(); i++)
		{
			if (neighbors.get(i) == this || ((Robot)neighbourhood.get(i)).isMoving) 
				continue;
			
			if (((Robot)neighbors.get(i)).validGradient)
			{
				neighValue = ((Robot)neighbors.get(i)).getGradientValue();
				if ( neighValue + 1 < current)
				{
					current = neighValue + 1;
					validGradient = true;
				}
					
			}
		}
		gradientValue = current;
		
	}
	
}
