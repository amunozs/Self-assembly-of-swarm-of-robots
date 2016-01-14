package Kilobots;

import sim.engine.*;
import sim.field.continuous.*;
import sim.util.*;

public class Robot implements Steppable
{
	double DESIRED_DISTANCE = 3.2;
	double orientation;
	int gradientValue=0;
	public boolean isStationary = false;
	public boolean isReference = false;
	public boolean isMoving = false;
	public boolean validGradient = false;
	public boolean isLocalized = false;
	public MutableDouble2D position;
	
	public int ID;
	
	public boolean inCollision = false;
	
	private double previousDistance = Double.MAX_VALUE;
	Bag neighborhood;
	Bag smallNeighborhood;
	
	Swarm swarm;
	Double2D me;
	Continuous2D space;
	
	public void setMoving (boolean isFollowing) {isMoving = isFollowing;}
	public boolean getMoving () {return isMoving;}
	
	public int getID () { return ID;}
	public boolean getIsReference() {return isReference;}
	public void setOrientation(double o) {orientation = o;}
	public double orientation2D () {return orientation;}
	public int getGradientValue () {return gradientValue;}
	public void setDesiredDistance(double d) {DESIRED_DISTANCE = d;}
	public double getDesiredDistance() { return DESIRED_DISTANCE;}
	public double getX() {return position.x;}
	public double getY() {return position.y;}
	
	private boolean moved = false;
	//Bag neighnorhood;
	
	//public int getNumNeigh() {return numNeig;}
	
	public void step(SimState state)
	{
		
		if(isReference) return;
		
		swarm = (Swarm) state;
		space = swarm.space;
		
		Double2D aux = space.getObjectLocation(this);
		
		
		boolean moved;
		if(me!=aux) moved = true;
		else moved = false;
		
		
		me = aux;
		neighborhood = space.getNeighborsWithinDistance(me, 10);	
		smallNeighborhood = space.getNeighborsExactlyWithinDistance(me, 4.5);	
		
		generateID();
		calculatePosition();
		calculateGradient();
		
		if (isMoving)
		{
			followEdge();
			if (swarm.checkPointInMap(me))
					isMoving = false;
					calculateGradient();
					validGradient = true;
					isStationary = true;
					return;
					
		}
		else
		{
			calculateGradient();
			isMoving = startMoving();
		}
		
		
		if (validMovement (me))
			inCollision = false;
		else
			inCollision = true;
		
		if (moved) validGradient = false;
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
		for (int i = 0; i<neighborhood.size(); i++)
		{
			if (neighborhood.get(i) == this || ((Robot)neighborhood.get(i)).isMoving) 
				continue;
			auxDistance = Swarm.getDistance (space.getObjectLocation(neighborhood.get(i)), me);
			
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
		//TODO arreglar calcular gradiente si los muevo a mano. Se va realimntando porque todos
		//     tienen validGradient
		
		Bag smallNeighborhood = space.getNeighborsExactlyWithinDistance(me, 4.5);
		int current = Integer.MAX_VALUE;
		int neighValue;
		for (int i = 0; i < smallNeighborhood.size(); i++)
		{
			if (smallNeighborhood.get(i) == this || ((Robot)smallNeighborhood.get(i)).isMoving) 
				continue;
			
			if (((Robot)smallNeighborhood.get(i)).validGradient)
			{
				neighValue = ((Robot)smallNeighborhood.get(i)).getGradientValue();
				if ( neighValue + 1 < current)
				{
					current = neighValue + 1;
					validGradient = true;
				}
					
			}
		}
		gradientValue = current;
		
	}
	
	private void calculatePosition()
	{
		MutableDouble2D position_me = new MutableDouble2D(0,0);
		Bag localized = new Bag(1);
		for (int i = 0; i < neighborhood.size(); i++)
		{
			if (neighborhood.get(i) == this || ! (((Robot)neighborhood.get(i)).isLocalized))
				continue;
			else
				localized.add(neighborhood.get(i));
		}
		if (localized.size() >= 3)
		{
			for (int i=0; i< localized.size(); i++)
			{
				double measured_distance = Swarm.getDistance(me, space.getObjectLocation(localized.get(i)));
				double c = Swarm.getDistance(position_me, ((Robot)localized.get(i)).position);
				Double2D v = new Double2D((position_me.x - ((Robot)localized.get(i)).position.x)/c,
						(position_me.y - ((Robot)localized.get(i)).position.y)/c);
				Double2D n = new Double2D ( ((Robot)localized.get(i)).position.x + measured_distance * v.x, 
						((Robot)localized.get(i)).position.y + measured_distance * v.y);
				position_me.x = position_me.x - (position_me.x - n.x)/4;
				position_me.y = position_me.y - (position_me.y - n.y)/4;
			}
		}
		
		position = position_me;
		if (validGradient) isLocalized = true;
	}
	
	private void generateID ()
	{
		for (int i = 0; i < neighborhood.size(); i++)
		{
			if (neighborhood.get(i) == this)
				continue;
			if (((Robot)neighborhood.get(i)).ID == ID)
			{
				ID = swarm.random.nextInt();
				i=0;
			}
		}
	}
	
	// Checks if the robot should start moving or remain stationary.
	private boolean startMoving ()
	{
		if (!validGradient) return false;
		// TODO cambiar esto por la posicion calculada
		if (swarm.checkPointInMap(me)) return false;
		boolean startMoving = false;
		int maxGradient = 0;
		int maxID = Integer.MIN_VALUE;
		
		// Obtain the biggest gradient regarding all close robots
		for (int i = 0; i<smallNeighborhood.size(); i++)
		{
			if (smallNeighborhood.get(i) == this || ((Robot)smallNeighborhood.get(i)).isStationary )
				continue;
			
			if ( ! ((Robot)smallNeighborhood.get(i)).validGradient)
				return false;
				
			if (((Robot)smallNeighborhood.get(i)).gradientValue > gradientValue)
				maxGradient = ((Robot)smallNeighborhood.get(i)).gradientValue;
		}
		
		// If its strictly the biggest gradient, it could possibly start moving
		if (gradientValue > maxGradient)
			startMoving = true;
		
		// If it has no neighbors with greater gradient and its the biggest ID, 
		// it could possibly start moving
		else if (gradientValue == maxGradient)
		{
			for (int i = 0; i<neighborhood.size(); i++)
			{
				if (neighborhood.get(i) == this || ((Robot)smallNeighborhood.get(i)).isStationary)
					continue;
				if (((Robot)neighborhood.get(i)).ID > gradientValue)
					maxID = ((Robot)neighborhood.get(i)).ID;
			}
			
			if (ID > maxID)
				startMoving = true;
		}
		
		// If there is already a neighbor moving, it can't start moving.
		if (startMoving)
		{
			for (int i = 0; i<neighborhood.size(); i++)
			{
				if (neighborhood.get(i) == this)
					continue;
				if (((Robot)neighborhood.get(i)).isMoving)
					startMoving = false;
			}
		}
		return startMoving;
	}
	
}
