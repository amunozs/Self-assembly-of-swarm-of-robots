package Kilobots;

import sim.util.*;

public class Line2D 
{
	public double intersection;
	public double slope;
	public Double2D point1;
	public Double2D point2;
	public boolean completing = false;
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
