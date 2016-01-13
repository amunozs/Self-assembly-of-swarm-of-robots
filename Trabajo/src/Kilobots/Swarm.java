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
	public int numRobots = 100;
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
	
	public void start()
	{
		super.start();
		space.clear();
		
		addRobot (new Double2D((space.getWidth() * 0.5 + 1.5),space.getHeight()*0.5), true);
		addRobot (new Double2D((space.getWidth() * 0.5 - 1.5),space.getHeight()*0.5), true);
		addRobot (new Double2D((space.getWidth() * 0.5),space.getHeight()*0.5 + 2.6), true);
		addRobot (new Double2D((space.getWidth() * 0.5),space.getHeight()*0.5 - 2.6), true);
		
		for (int i = 0; i < numRobots; i++)
		{
			addRobot(new Double2D(space.getWidth() * 0.5 + (random.nextDouble() - 0.5)*10,
							space.getHeight() * 0.5 + (random.nextDouble() - 0.5)*10));
		}
	}/*
		// Read the map
		BufferedImage im = null;
		try {
			 im = ImageIO.read(new File("map.jpg"));
		} catch (IOException e) {
			e.printStackTrace();
		}
		// Convert to to a binary image
		map = new BufferedImage(im.getWidth(), im.getHeight(), BufferedImage.TYPE_BYTE_BINARY);
	}
	
	public boolean checkPointInMap (Double2D point)
	{
		int mapWidth = map.getWidth();
		int mapHeight = map.getHeight();
		double spaceWidth = space.getWidth();
		double spaceHeight = space.getHeight();
		
	}*/
	
	private void addRobot(Double2D position) {addRobot(position, false);}

	private void addRobot(Double2D position, boolean isReference)
	{
		Robot robot = new Robot();
		robot.isReference = isReference;
		robot.validGradient = isReference;
		robot.setOrientation(random.nextDouble() * 6.28319);
		space.setObjectLocation(robot, position);
		schedule.scheduleRepeating(robot);
	}
	
	
}
