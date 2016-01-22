package Kilobots;

import sim.engine.*;
import sim.field.continuous.*;
import sim.util.*;

/**
 * A kilobot
 * @author Álvaro Muñoz Serrano
 *
 */
public class Robot implements Steppable
{	
	public enum State {WAITING, MOVING2, STOPPED, MOVING, LINE} 
	
	// Actual and next states
	public State actual_state = State.WAITING;
	public State next_state = State.WAITING;
	
	// Desired distance for following the edge
	double DESIRED_DISTANCE = 3.2;

	// If the robot is one of the references
	public boolean isReference = false;
	
	// If the gradient and location are valid or the reference has lost.
	public boolean validGradient = false;
	public boolean isLocalized = false;
	
	// position, orientation (in radians), ID and gradient.
	public MutableDouble2D position = new MutableDouble2D (0,0);
	double orientation;
	public int ID;
	int gradientValue=0;
	
	// If it is colliding with other robot
	public boolean inCollision = false;
	
	// Previous minimum distance for following edge
	private double previousDistance = Double.MAX_VALUE;
	
	// Neighborhoods for gradient, position, start moving, etc.
	Bag neighborhood;
	Bag smallNeighborhood;
	
	// Information of the swarm
	Swarm swarm;
	Continuous2D space;
	
	// Actual and next positions
	Double2D me;
	Double2D next_me;
	
	// Methods for the GUI
	public int getID () { return ID;}
	public boolean getIsReference() {return isReference;}
	public void setOrientation(double o) {orientation = o;}
	public double orientation2D () {return orientation;}
	public int getGradientValue () {return gradientValue;}
	//public double getX() {return position.x;}
	//public double getY() {return position.y;}
	
	public State getState () {return actual_state;}
	public State getNextState () {return next_state;}
	
	private boolean moved = false;

	public void step(SimState state)
	{
		// If is one of the reference robots don't do anything.
		if(isReference) return;
	
		
		swarm = (Swarm) state;
		space = swarm.space;
		
		Double2D aux = space.getObjectLocation(this);
		
		// Check if the robot have been moved by the user.
		boolean moved;
		if(me!=aux) moved = true;
		else moved = false;
		
		me = aux;
		neighborhood = space.getNeighborsWithinDistance(me, 10);	
		smallNeighborhood = space.getNeighborsExactlyWithinDistance(me, 3.3);	
		
		// Calculate the ID, gradient and position (if required)
		generateID();
		if (swarm.calculatePositions) 
			calculatePosition();
		
		calculateGradient();
		
		// Actualize state
		actual_state = next_state;
		next_state = actual_state;
		next_me = me;
		
		// Move if required.
		switch(actual_state) 
		{
		case MOVING2: 	followEdge();
						space.setObjectLocation(this, next_me);
						break;
						
		case MOVING: 	followEdge();
						space.setObjectLocation(this, next_me);
						break;
		}
		
		// Calculate next state.
		switch(actual_state) 
		{
			case WAITING: 	if (startMoving()) next_state = State.MOVING;
							break;
			
			case MOVING: 	if (becomeStationary(next_me)) 
							{
								next_state = State.STOPPED;
								swarm.numRobotsInZone++;
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
		
		if (moved) validGradient = false;
	}
	
	/**
	 * Calculate the next position if moving forward
	 */
	private void moveForward ()
	{
		MutableDouble2D nextPosition = new MutableDouble2D();
		
		// Advance 0.2 points each step
		nextPosition.addIn(Math.cos(orientation)*0.2, Math.sin(orientation)*0.2);
		nextPosition.addIn(me);
		
		Double2D n = null;
		if (!swarm.calculatePositions)
			 n = new Double2D(nextPosition);
		else
			n = new Double2D(position.x + Math.cos(orientation)*0.1, position.y + Math.sin(orientation)*0.1);
		next_me = n;
	}
	
	/**
	 * Checks if the robot is in position to stop moving in the next step.
	 * @param nextPosition The next position.
	 * @return true if it should go to STOPPED state.
	 */
	private boolean becomeStationary ( Double2D nextPosition )
	{
		// If it is not in the zone, return false.
		if (swarm.checkPointInMap(me) && !swarm.calculatePositions ||
				swarm.checkPointInMap(new Double2D(position)) && swarm.calculatePositions)
		{
			// If it is in the zone, check all neighbors
			for (int i = 0; i <smallNeighborhood.size(); i++)
			{
				// If the next position is out of the zone, return true.
				if (!swarm.checkPointInMap(nextPosition))
					return true;
					
				// If there is a robot in the small neighborhood with same gradient, return true.
				if (((Robot)smallNeighborhood.get(i)).actual_state == State.STOPPED && 
						((Robot)smallNeighborhood.get(i)).gradientValue == gradientValue )
				{

					return true;
				}
					
			}
		}
		// By default return false
		return false;
	}
	
	/**
	 * Checks if the current robot will go to LINE state
	 * @return If the robot should join the line
	 */
	private boolean becomeCreateLine ()
	{
		if (swarm.checkPointInLine (me) && !swarm.checkPointInMap(me))
		{
			return true;
		}
		else return false;
	}
	
	/**
	 * Checks if the current robot will go to WAITING state
	 * @return If the robot should leave the line
	 */
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
				if (((Double2D)space.getObjectLocation(smallNeighborhood.get(i))).x > me.x +1 &&
						((Double2D)space.getObjectLocation(smallNeighborhood.get(i))).y > me.y + 1)
				{	
					((Line2D)swarm.vectors.get(swarm.actual_line)).completing = false;
					((Line2D)swarm.vectors.get(swarm.actual_line)).completed = true;
					return true;
				}
				//if (swarm.getAngle(((Double2D)space.getObjectLocation(smallNeighborhood.get(i)))) < )
		}
		return false;
	}
	
	/**
	 * Checks if the robot will complete a loop on the swarm
	 * @param nextPosition The next position of the robot.
	 * @return true if it completes a loop false if not
	 */
	private boolean checkLoop (Double2D nextPosition)
	{
		if (me.x >= space.getWidth() * 0.5 && me.y <= space.getHeight()*0.5 + 1 &&
				nextPosition.y > space.getHeight() * 0.5 + 1)
			return true;
		return false;
	}
	
	/**
	 * Checks if the next movement will produce a collision.
	 * @param nextPosition Position to check.
	 * @return if it is in collision
	 */
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
	
	/**
	 * Rotate in the desired direction.
	 * @param direction True for counterclockwise, false for clockwise
	 */
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
	
	/**
	 * Implement the logic of following the edge of the swarm
	 */
	private void followEdge ()
	{
		double distance = Double.MAX_VALUE;
		double auxDistance;
		
		// Get the closest neighbor.
		for (int i = 0; i<neighborhood.size(); i++)
		{
			if (neighborhood.get(i) == this || ((Robot)neighborhood.get(i)).actual_state == State.MOVING2 || 
					((Robot)neighborhood.get(i)).actual_state == State.MOVING) 
				continue;
			auxDistance = Swarm.getDistance (space.getObjectLocation(neighborhood.get(i)), me);
			
			if(auxDistance < distance)
				distance = auxDistance;
		}
		
		// If the distance is lower than desired, move forward and rotate 
		// counterclockwise if the distance at the step before was higher.
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
		
		// If the distance is higher than desired, move forward and rotate 
		// clockwise if the distance at the step before was lower.
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
	
	/**
	 * Calculate the value of the gradient based on the neighbors.
	 */
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
			if (smallNeighborhood.get(i) == this) 
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
	
	/**
	 * Calculate the position of the robot by trilateration.
	 */
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
	/**
	 * Generate the random locally unique ID
	 */
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
	
	/**
	 *  Checks if the robot should start moving or remain stationary.
	 * @return if the robot should start moving
	 */
	private boolean startMoving ()
	{
		if (!validGradient || actual_state != State.WAITING) return false;

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
