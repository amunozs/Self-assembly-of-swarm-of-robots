package Kilobots;

import sim.engine.*;
import sim.field.continuous.*;
import sim.util.*;

public class Robot implements Steppable
{	
	public enum State {WAITING, MOVING2, STOPPED, MOVING, LINE} 
	
	public State actual_state = State.WAITING;
	public State next_state = State.WAITING;
	
	double DESIRED_DISTANCE = 3.2;
	double orientation;
	int gradientValue=0;
	
	//public boolean isStationary = false;
	public boolean isReference = false;
	//public boolean isMoving = false;
	//public boolean isCreatingLine = false;
	
	public boolean validGradient = false;
	public boolean isLocalized = false;
	public MutableDouble2D position = new MutableDouble2D (0,0);
	
	
	public int ID;
	
	public boolean inCollision = false;
	
	
	private double previousDistance = Double.MAX_VALUE;
	Bag neighborhood;
	Bag smallNeighborhood;
	
	Swarm swarm;
	Double2D me;
	Double2D next_me;
	Continuous2D space;
	
	//public void setMoving (boolean isFollowing) {isMoving = isFollowing;}
	//public boolean getMoving () {return isMoving;}
	
	public int getID () { return ID;}
	public boolean getIsReference() {return isReference;}
	public void setOrientation(double o) {orientation = o;}
	public double orientation2D () {return orientation;}
	public int getGradientValue () {return gradientValue;}
	public void setDesiredDistance(double d) {DESIRED_DISTANCE = d;}
	public double getDesiredDistance() { return DESIRED_DISTANCE;}
	public double getX() {return position.x;}
	public double getY() {return position.y;}
	
	public State getState () {return actual_state;}
	public State getNextState () {return next_state;}
	
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
		smallNeighborhood = space.getNeighborsExactlyWithinDistance(me, 3.3);	
		
		generateID();
		if (swarm.calculatePositions) 
			calculatePosition();
		
		calculateGradient();
		
		actual_state = next_state;
		next_state = actual_state;
		next_me = me;
		
		switch(actual_state) 
		{
		case MOVING2: 	followEdge();
						space.setObjectLocation(this, next_me);
						break;
						
		case MOVING: 	followEdge();
						space.setObjectLocation(this, next_me);
						break;
		}
		
		switch(actual_state) 
		{
			case WAITING: 	if (startMoving()) next_state = State.MOVING;
							break;
			
			case MOVING: 	if (becomeStationary(next_me)) 
							{
								next_state = State.STOPPED;
								swarm.numRobotsInZone++;
								//next_me = me;
							}
							else if (checkLoop(next_me)) next_state = State.MOVING2;
							else space.setObjectLocation(this, next_me);
							break;
			
			case MOVING2: 	if (swarm.checkPointInMap(next_me)) next_state = State.MOVING;	
							else if (swarm.checkPointInLine(next_me)) next_state = State.LINE;
							break;
							
			
			
			case LINE: 		if (leaveLine()) next_state = State.WAITING;
							break;
		}
		
		
		/*if (isMoving)
			followEdge();

		else
			isMoving = startMoving();
		
		
		if (validMovement (me))
			inCollision = false;
		else
			inCollision = true;*/
		
		if (moved) validGradient = false;
	}
	
	private void moveForward ()
	{
		// TODO 10min Limpiar esto.
		MutableDouble2D nextPosition = new MutableDouble2D();
		nextPosition.addIn(Math.cos(orientation)*0.2, Math.sin(orientation)*0.2);
		nextPosition.addIn(me);
		
		//if(validMovement(new Double2D(nextPosition)))
		Double2D n = null;
		Double2D movement = new Double2D(nextPosition);
		if (!swarm.calculatePositions)
			 n = new Double2D(nextPosition);
		else
			// TODO 20min la orientación se supone que no la se
			n = new Double2D(position.x + Math.cos(orientation)*0.1, position.y + Math.sin(orientation)*0.1);
			
		/*if (!becomeStationary(n)/* && !becomeCreateLine())*/
		/*	space.setObjectLocation(this, movement);
		else 
			swarm.numRobotsInZone++;*/
		next_me = n;
	}
	
	private boolean becomeStationary ( Double2D nextPosition )
	{
		// TODO meter calculate positions
		if (swarm.checkPointInMap(me) && !swarm.calculatePositions ||
				swarm.checkPointInMap(new Double2D(position)) && swarm.calculatePositions)
		{
			for (int i = 0; i <smallNeighborhood.size(); i++)
			{
				if (!swarm.checkPointInMap(nextPosition))
				{
					//isStationary = true;
					//isMoving = false;
					return true;
				}
					
				if (((Robot)smallNeighborhood.get(i)).actual_state == State.STOPPED && 
						((Robot)smallNeighborhood.get(i)).gradientValue == gradientValue )
				{
					//isStationary = true;
					//isMoving = false;
					return true;
				}
					
			}
		}
		
		return false;
	}
	
	private boolean becomeCreateLine ()
	{
		if (swarm.checkPointInLine (me) && !swarm.checkPointInMap(me))
		{
			//isMoving = false;
			//isCreatingLine = true;
			return true;
		}
		else return false;
	}
	
	private boolean leaveLine ()
	{
		for (int i = 0; i< smallNeighborhood.size(); i++)
		{
			if (((Robot)smallNeighborhood.get(i)).actual_state == State.WAITING)
				return true;
			
			else if (smallNeighborhood.get(i) == this || 
					((Robot)smallNeighborhood.get(i)).actual_state != State.MOVING2 ) 
				continue;
			else
				if (((Double2D)space.getObjectLocation(smallNeighborhood.get(i))).x > me.x &&
						((Double2D)space.getObjectLocation(smallNeighborhood.get(i))).y > me.y)
				{	
					((Line2D)swarm.vectors.get(swarm.actual_line)).completing = false;
					((Line2D)swarm.vectors.get(swarm.actual_line)).completed = true;
					return true;
				}
		}
		return false;
	}
	
	private boolean checkLoop (Double2D nextPosition)
	{
		if (me.x >= space.getWidth() * 0.5 && me.y <= space.getHeight()*0.5 + 1 &&
				nextPosition.y > space.getHeight() * 0.5 + 1)
			return true;
		return false;
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
			if (neighborhood.get(i) == this || ((Robot)neighborhood.get(i)).actual_state == State.MOVING2 || 
					((Robot)neighborhood.get(i)).actual_state == State.MOVING) 
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
			if (smallNeighborhood.get(i) == this/* || ((Robot)smallNeighborhood.get(i)).actual_state == State.MOVING2 
					|| ((Robot)smallNeighborhood.get(i)).actual_state == State.MOVING*/) 
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
		MutableDouble2D previous_me = new MutableDouble2D(0,0);
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
			for (int s = 0; s<1000; s++)
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
		if (!validGradient || actual_state != State.WAITING) return false;
		// TODO cambiar esto por la posicion calculada?
		//if (swarm.checkPointInMap(me)) return false;
		boolean startMoving = false;
		int maxGradient = 0;
		int maxID = Integer.MIN_VALUE;
		
		// Obtain the biggest gradient regarding all close robots
		for (int i = 0; i<neighborhood.size(); i++)
		{
			if (neighborhood.get(i) == this || ((Robot)neighborhood.get(i)).actual_state != State.WAITING)
				continue;
			
			if ( ! ((Robot)neighborhood.get(i)).validGradient)
				return false;
				
			if (((Robot)neighborhood.get(i)).gradientValue > maxGradient)
				maxGradient = ((Robot)neighborhood.get(i)).gradientValue;
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
				if (neighborhood.get(i) == this || ((Robot)neighborhood.get(i)).actual_state != State.WAITING)
					continue;
				if (((Robot)neighborhood.get(i)).gradientValue == gradientValue && 
						((Robot)neighborhood.get(i)).ID > maxID)
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
				if (((Robot)neighborhood.get(i)).actual_state == State.MOVING2 || 
						((Robot)neighborhood.get(i)).actual_state == State.MOVING)
					startMoving = false;
			}
		}
		return startMoving;
	}
	
}
