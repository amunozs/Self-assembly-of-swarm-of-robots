package Kilobots;

import sim.util.*;

public class Line2D 
{
	public double intersection;
	public double slope;
	
	Line2D (double i, double s)
	{
		intersection = i;
		slope = s;
	}
	
	Line2D (Double2D point1, Double2D point2)
	{
		slope = (point2.y - point1.y) / (point2.x - point1.x);
		intersection = point1.y - point1.x * slope;
	}
	
	public boolean isPointInLine (Double2D point, double margin)
	{
		if (point.y > slope * point.x + intersection - margin && 
				point.y < slope * point.x + intersection + margin)
			return true;
		
		else
			return false;
	}
}
