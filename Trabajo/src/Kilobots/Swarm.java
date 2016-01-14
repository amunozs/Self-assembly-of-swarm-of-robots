package Kilobots;

import java.awt.image.BufferedImage;
import java.io.File;
import java.io.IOException;

import javax.imageio.ImageIO;

import sim.engine.*;
import sim.util.*;
import sim.field.continuous.*;

public class Swarm extends SimState
{
	public Continuous2D space = new Continuous2D(10,1000,1000);
	public int numRobots = 10;
	public BufferedImage map;
	
	public Swarm(long seed) {super(seed);}	

	static public boolean checkCollision(Double2D a, Double2D b)
	{
		double xDif = a.x - b.x;
		double yDif = a.y - b.y;
		double distanceSquared = xDif * xDif + yDif * yDif;
		return (distanceSquared < 9);
	}
	
	static public double getDistance(Double2D a, Double2D b)
	{
		double xDif = a.x - b.x;
		double yDif = a.y - b.y;
		return Math.sqrt(xDif * xDif + yDif * yDif);
	}
	
	static public double getDistance(MutableDouble2D a, MutableDouble2D b)
	{
		double xDif = a.x - b.x;
		double yDif = a.y - b.y;
		return Math.sqrt(xDif * xDif + yDif * yDif);
	}
	
	public void start()
	{
		super.start();
		space.clear();
		
		addRobot (new Double2D((space.getWidth() * 0.5 + 1.5),space.getHeight()*0.5), true, 1);
		addRobot (new Double2D((space.getWidth() * 0.5 - 1.5),space.getHeight()*0.5), true, 2);
		addRobot (new Double2D((space.getWidth() * 0.5),space.getHeight()*0.5 + 2.6), true, 3);
		addRobot (new Double2D((space.getWidth() * 0.5),space.getHeight()*0.5 - 2.6), true, 4);
		
		for (int i = 0; i < numRobots; i++)
		{
			addRobot(new Double2D(space.getWidth() * 0.5 + (random.nextDouble() - 0.5)*10,
							space.getHeight() * 0.5 + (random.nextDouble() - 0.5)*10));
		}
	//}
		// Read the map
		BufferedImage im = null;
		try {
			 im = ImageIO.read(new File("map.jpg"));
			 map = new BufferedImage(im.getWidth(), im.getHeight(), BufferedImage.TYPE_BYTE_BINARY);
		} catch (IOException e) {
			e.printStackTrace();
		}
		// Convert to to a binary image
		
	}
	
	/*public boolean checkPointInMap (Double2D point)
	{
		int mapWidth = map.getWidth();
		int mapHeight = map.getHeight();
		double spaceWidth = space.getWidth();
		double spaceHeight = space.getHeight();
		
	}*/
	
	private void addRobot(Double2D position) {addRobot(position, false, 0);}

	private void addRobot(Double2D position, boolean isReference, int n)
	{
		Robot robot = new Robot();
		robot.isReference = isReference;
		robot.validGradient = isReference;
		robot.isLocalized = isReference;
		if (isReference)
		{
			
			if (n==1)
				robot.position = new MutableDouble2D (1.5,0);
			else if (n==2)
				robot.position = new MutableDouble2D (-1.5,0);
			else if (n==3)
				robot.position = new MutableDouble2D (0,2.6);
			else if (n==4)
				robot.position = new MutableDouble2D (0,-2.6);
		}
		
		
		
		robot.setOrientation(random.nextDouble() * 6.28319);
		space.setObjectLocation(robot, position);
		schedule.scheduleRepeating(robot);
	}
	
	public boolean checkPointInMap(Double2D point)
	{
		if (point.x <= 500.05 && point.x >= -500.05 && point.y >= 500)
			return true;
		return false;
	}
	
	
}
