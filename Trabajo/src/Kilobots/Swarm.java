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
	public boolean show_colors = false;
	public void setColors (boolean colors) { show_colors = colors;}
	public boolean getColors (){return show_colors;}
	
	public Continuous2D space = new Continuous2D(10,210,210);
	public int numRobots = 500;
	public BufferedImage map;
	public String imgFile = "Islands2.png";
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
	
	public int getNumIslands () {return vectors.size();}
	
	public Bag vectors = new Bag(0);
	//public int[] rgb = new int [20];
	public double[] minDist = new double [20];
	public int actual_line;
	
	
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
	
	public double getAngle(Double2D point)
	{
		return Math.atan( - (point.y - space.getHeight() * 0.5) / (point.x - space.getWidth() * 0.5));
	}
	
	public void start()
	{
		super.start();
		space.clear();
		
		readMap();
		calculateArea();
		space = new Continuous2D(10,map.getWidth() * 2 + 10 ,map.getWidth() * 2 + 10);
		
		addRobot (new Double2D((space.getWidth() * 0.5 + 1.5),space.getHeight()*0.5), true, 1);
		addRobot (new Double2D((space.getWidth() * 0.5 - 1.5),space.getHeight()*0.5), true, 2);
		addRobot (new Double2D((space.getWidth() * 0.5),space.getHeight()*0.5 + 2.6), true, 3);
		addRobot (new Double2D((space.getWidth() * 0.5),space.getHeight()*0.5 - 2.6), true, 4);
		
		int width = (int) Math.sqrt(numRobots);
		double x = space.getWidth()*0.5 - width*3.1;
		double y = space.getHeight()*0.5 + 4.15;
		
		for (int c = 0; c< width; c++)
		{
			for (int i = 0; i < width; i++)
			{
				addRobot(new Double2D(x,y));
				x = x + 3.1;
			}
			x = space.getWidth()*0.5 - width*3.1;
			y = y + 3.1;
		}

	}
	
	private BufferedImage getImage(String filename)
	{
		try {
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
		{
			
			if (n==1)
				// robot.position = new MutableDouble2D (1.5,0);
				robot.gradientValue = 0;
			else if (n==2)
				// robot.position = new MutableDouble2D (-1.5,0);
				robot.gradientValue = 1;
			else if (n==3)
				//robot.position = new MutableDouble2D (0,2.6);
				robot.gradientValue = 1;
			else if (n==4)
				//robot.position = new MutableDouble2D (0,-2.6);
				robot.gradientValue = 1;
		}
		
		
		
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
			if (((Line2D)vectors.get(i)).completing)
			{
				if (((Line2D)vectors.get(i)).isPointInLine(point, 0.3))
				{
					actual_line =  i;
					return true;
				}
					
				else 
					return false;
			}
		}
		for (int i = 0; i<vectors.size(); i++)
		{
			if (((Line2D)vectors.get(i)).isPointInLine(point, 0.3) && ! ((Line2D)vectors.get(i)).completed)
			{
				((Line2D)vectors.get(i)).completing = true;
				actual_line =  i;
				return true;
			}
			
		}
		return false;
	}
	
	private void readMap()
	{
		int aux_rgb;
		int it = 0;
		Bag points = new Bag(0);

		int[] rgb = new int[10];
		rgb[0] = 0;
		map = getImage("/Resources/" + imgFile);
		
		vectors = new Bag(0);
		for (int x = 0; x< map.getWidth(); x++)
		{
			for (int y = 0; y<map.getHeight(); y++)
			{
				aux_rgb = map.getRGB(x, y);
				
				if (aux_rgb != Color.white.getRGB() && aux_rgb != Color.black.getRGB())
				{
					rgb[it] = aux_rgb;
					it++;
					points.add(new Double2D(x + space.width*0.5, y + space.height*0.5 - map.getHeight()));
				}
			}
		}

		for (int i = 0; i< (points.size()-1); i++)
		{
			for (int c = i+1; c< points.size(); c++ )
				if (rgb[i] == rgb[c])
				{
					vectors.add(new Line2D ((Double2D)points.get(i), (Double2D)points.get(c)));
					System.out.println("LINE:");
					System.out.println("x=" + ((Double2D)points.get(i)).x + ", y=" + ((Double2D)points.get(i)).y);
					System.out.println("x=" + ((Double2D)points.get(c)).x + ", y=" + ((Double2D)points.get(c)).y);
					break;
				}
		}
		
	}
}
