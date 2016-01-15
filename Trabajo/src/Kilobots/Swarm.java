package Kilobots;

import java.awt.Color;
import java.awt.image.BufferedImage;
import java.io.IOException;
import java.io.InputStream;

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
		
		int width = (int) Math.sqrt(numRobots);
		double x = space.getWidth()*0.5 - width*0.5*3.1;
		double y = space.getHeight()*0.5 + 4.15;
		
		for (int c = 0; c< width; c++)
		{
			for (int i = 0; i < width; i++)
			{
				addRobot(new Double2D(x,y));
				x = x + 3.1;
			}
			x = space.getWidth()*0.5 - width*0.5*3.05;
			y = y + 3.1;
		}
	//}
		// Read the map
		map = getImage("/Resources/prueba2.png");
		
		// Convert to to a binary image
		
	}
	
	private BufferedImage getImage(String filename)
	{
		try {
			// map = ImageIO.read(new File("prueba1.png"));
			 //map = new BufferedImage(im.getWidth(), im.getHeight(), BufferedImage.TYPE_BYTE_BINARY);
			InputStream in = getClass().getResourceAsStream(filename);
			return ImageIO.read(in);
		} catch (IOException e) {
			e.printStackTrace();
		}
		return null;
	}
	
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
		/*if (point.x <= 500.05 && point.x >= 499.95 && point.y >= 500)
			return true;*/

		int x = (int) (point.x - space.getWidth()*0.5);
		int y = (int) (point.y - space.getHeight()*0.5 + map.getHeight());
		
		if (x < 0 || y < 0 || x >= map.getWidth() || y >= map.getHeight())
			return false;
		else
			if (map.getRGB(x, y) == Color.black.getRGB())
				return true;
			else 
				return false;
	}
	
	
}
