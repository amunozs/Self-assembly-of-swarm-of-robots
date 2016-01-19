package Kilobots;

import sim.engine.*;
import sim.display.*;
import sim.portrayal.DrawInfo2D;
import sim.portrayal.Inspector;
import sim.portrayal.continuous.*;
import sim.portrayal.simple.*;

import java.awt.Color;
import java.awt.Graphics2D;
import java.awt.color.*;

import javax.swing.JFrame;

import SocialNetwork.Student;

public class SwarmWithUI extends GUIState
{
	
	public Display2D display;
	public JFrame displayFrame;
	ContinuousPortrayal2D spacePortrayal = new ContinuousPortrayal2D();
	
	public static void main (String[] args)
	{
		SwarmWithUI vid = new SwarmWithUI();
		Console c = new Console(vid);
		c.setVisible(true);
	}
	
	
	public void start()
	{
		super.start();
		setupPortrayals();
	}

	public void load(SimState state)
	{
	super.load(state);
	setupPortrayals();
	}

	public void setupPortrayals()
	{
		Swarm swarm = (Swarm) state;
		// tell the portrayals what to portray and how to portray them
		spacePortrayal.setField( swarm.space );
		spacePortrayal.setPortrayalForAll(
				new MovablePortrayal2D(
						new OrientedPortrayal2D(
						new CircledPortrayal2D(
								new LabelledPortrayal2D(
										
											new OvalPortrayal2D(Color.gray, 3, true){
												public void draw(Object object, Graphics2D graphics, DrawInfo2D info)
												{
													Robot robot = (Robot)object;
													if (robot.isReference)
														paint = Color.green;
													else if (robot.isCreatingLine)
														paint = Color.blue;
													else if (robot.isStationary)
														paint = Color.black;
													/*else if (!robot.validGradient)
														paint = Color.orange;*/
													else
														paint = Color.gray;
													super.draw(object, graphics, info);
												}
									
											},
											
										5.0, null, Color.black, true),
								0, 5.0, Color.blue, true),
						0, 5.0, Color.red)));
		// reschedule the displayer
		
		display.reset();
		display.setBackdrop(Color.white);
		// redraw the display
		display.repaint();
	}
	
	public void init(Controller c)
	{
		super.init(c);
		display = new Display2D(600,600,this);
		display.setClipping(false);
		displayFrame = display.createFrame();
		displayFrame.setTitle("Schoolspace Display");
		c.registerFrame(displayFrame); // so the frame appears in the "Display" list
		displayFrame.setVisible(true);
		display.attach( spacePortrayal, "space" );
	}

	
	public void quit()
	{
		super.quit();
		if (displayFrame!=null) displayFrame.dispose();
		displayFrame = null;
		display = null;
	}
	
	public Object getSimulationInspectedObject() { return state; }
	public Inspector getInspector()
	{
		Inspector i = super.getInspector();
		i.setVolatile(true);
		return i;
	}
	
	
	
	public SwarmWithUI() { super(new Swarm(System.currentTimeMillis())); }
	public SwarmWithUI(SimState state) { super(state); }
	public static String getName() {return "Self assembling robots";}
}
