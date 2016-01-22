package Kilobots;

import sim.util.*;

/**
 * A segment defined by two points.
 * @author Álvaro Muñoz Serrano
 *
 */
public class Line2D 
{
	public double intersection;
	public double slope;
	public Double2D point1;
	public Double2D point2;
	
	// If the line has already been completed or 
	public boolean completing = false;
	// If the line is in process of being created.
	public boolean completed = false;
	

	Line2D (Double2D p1, Double2D p2)
	{
		if (p1.x < p2.x)
		{
			point1 = p1;
			point2 = p2;
		}
		else
		{
			point2 = p1;
			point1 = p2;
		}
		
		slope = (point2.y - point1.y) / (point2.x - point1.x);
		intersection = point1.y - point1.x * slope;
	}
	
	/**
	 * If a given point is in the line with an error margin
	 * @param point Point to check
	 * @param margin Error margin
	 * @return True if it is in the line, false if not.
	 */
	public boolean isPointInLine (Double2D point, double margin)
	{
		if (point.y + margin > slope * point.x + intersection && 
				point.y - margin < slope * point.x + intersection &&
				point.x >= point1.x && point.x <= point2.x)
			return true;
		
		else
			return false;
	}
}
