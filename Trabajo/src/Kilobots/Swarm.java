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
	public Continuous2D space = new Continuous2D(10,170,170);
	public int numRobots = 500;
	public BufferedImage map;
	public String imgFile = "prueba3.png";
	public boolean calculatePositions = false;
	private int totalArea;
	public int numRobotsInZone = 0;
	
	public Swarm(long seed) {super(seed);}	
	
	public int getNumRobots () { return numRobots;}
	public void setNumRobots (int n) {numRobots = n;}
	public String getImage () { return imgFile;}
	public void setImage (String file) {imgFile = file;}
	public boolean getCalulatePositions (){return calculatePositions;}
	public void setCalulatePositions (boolean n) {calculatePositions = n;}
	public double getSlope ()
	{
		if (vectors == null) return 0;
		else return ((Line2D)vectors.get(0)).slope;
		
	}
	public double getIntersection ()
	{
		if (vectors == null) return 0;
		else return ((Line2D)vectors.get(0)).intersection;
		
	}
	
	public Bag vectors = null;
	
	public double getArea ()
	{
		return ((double)numRobotsInZone)*7.07832 / (double)totalArea;
	}
	
	private void calculateArea ()
	{
		totalArea = 0;
		for (int i = 0; i<map.getWidth(); i++)
		{
			for (int c = 0; c<map.getHeight(); c++)
			{
				if (map.getRGB(i,c) == Color.black.getRGB())
					totalArea++;
			}
		}
	}
	
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
		map = getImage("/Resources/" + imgFile);
		calculateArea();
		
		// Convert to to a binary image
		
		vectors = new Bag(0);
		vectors.add(new Line2D(new Double2D (85,85), new Double2D (90,80)) );
		
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
			robot.position = new MutableDouble2D(position);
		/*{
			
			if (n==1)
				robot.position = new MutableDouble2D (1.5,0);
			else if (n==2)
				robot.position = new MutableDouble2D (-1.5,0);
			else if (n==3)
				robot.position = new MutableDouble2D (0,2.6);
			else if (n==4)
				robot.position = new MutableDouble2D (0,-2.6);
		}*/
		
		
		
		robot.setOrientation(random.nextDouble() * 6.28319);
		space.setObjectLocation(robot, position);
		schedule.scheduleRepeating(robot);
	}
	
	public boolean checkPointInMap(Double2D point)
	{

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
	public boolean checkPointInLine (Double2D point)
	{
		if (point.x < space.getWidth() * 0.5 || point.y > space.getHeight() * 0.5) 
			return false;
		for (int i = 0; i<vectors.size(); i++)
		{
			if (((Line2D)vectors.get(i)).isPointInLine(point, 0.1))
			return true;
		}
		return false;
	}
	
	
}
