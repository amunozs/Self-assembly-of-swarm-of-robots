package GameOfLife;

import sim.engine.*;
import sim.display.*;
import sim.portrayal.grid.*;
import java.awt.*;
import javax.swing.*;


public class GameOfLifeWithUI extends GUIState
{
	public Display2D display;
	public JFrame displayFrame;
	public GameOfLifeWithUI()
	{
		super(new GameOfLife(System.currentTimeMillis()));
	}
	
	public GameOfLifeWithUI(SimState state)
	{
		super(state);
	}
	
	FastValueGridPortrayal2D gridPortrayal = new FastValueGridPortrayal2D();
	public void setupPortrayals()
	{
		// tell the portrayals what to portray and how
		// to portray them
		gridPortrayal.setField(((GameOfLife)state).grid);
		gridPortrayal.setMap(new sim.util.gui.SimpleColorMap(
		new Color[] {new Color(0,0,0,0), Color.blue}));
	}
	
	private void setup()
	{
		setupPortrayals(); // set up our portrayals
		display.reset(); // reschedule the displayer
		display.repaint(); // redraw the display
	}
	
	public void start()
	{
		super.start();
		setup();
	}
	
	public void load(SimState state)
	{
		super.load(state);
		// we now have a new grid. Set up the portrayals to reflect this
		setup();
	}
	
	public void init(Controller c)
	{
		super.init(c);
		// Make the Display2D. We’ll have it display stuff later.
		GameOfLife gol = (GameOfLife)state;
		display = new Display2D(gol.gridWidth * 4,
		gol.gridHeight * 4,this,1);
		displayFrame = display.createFrame();
		// register the frame so it appears in the "Display" list
		c.registerFrame(displayFrame);
		displayFrame.setVisible(true);
		// attach the portrayals
		display.attach(gridPortrayal,"Life");
		// specify the backdrop color -- what gets painted behind the displays
		display.setBackdrop(Color.black);
	}
	
	public static void main(String[] args)
	{
		new GameOfLifeWithUI().createController();
	}
}

