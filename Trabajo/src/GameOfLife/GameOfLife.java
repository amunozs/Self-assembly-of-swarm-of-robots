package GameOfLife;

import sim.engine.*;
import sim.field.grid.*;

public class GameOfLife extends SimState implements Steppable{

	public int gridWidth = 40;
	public int gridHeight = 20;
	public IntGrid2D grid = null;
	
	public GameOfLife(long seed)
	{
		super(seed);
	}
	
	public void start()
	{
		super.start();
		grid = new IntGrid2D(gridWidth, gridHeight);
		seedGrid();
		schedule.scheduleRepeating(this);
	}
	
	private void seedGrid()
	{
		for(int i=0; i<gridWidth; i++)
		{
			for (int c=0; c<gridHeight; c++)
			{
				grid.field[i][c] = random.nextInt(2);
			}
		}
	}
	
	public void step(SimState state)
	{
		IntGrid2D aux = new IntGrid2D(gridWidth, gridHeight);
		int numAliveNeighs;
		for(int i=0; i<gridWidth; i++)
		{
			for (int c=0; c<gridHeight; c++)
			{
				aux.field[i][c] = grid.field[i][c];
			}
		}
		
		for(int i=0; i<gridWidth; i++)
		{
			for (int c=0; c<gridHeight; c++)
			{
				numAliveNeighs = 0;
				for(int x=i-1; x<=i+1; x++)
				{
					if(x>=gridWidth || x<0) continue;
					for(int y=c-1; y<=c+1; y++)
					{
						if(y>=gridHeight || y<0) continue;
						if(aux.field[x][y] == 1)
							numAliveNeighs++;
					}
				}
				if (grid.field[i][c] == 1 && (numAliveNeighs < 2 || numAliveNeighs >3)) grid.field[i][c] = 0;
				else if (grid.field[i][c] == 0 && numAliveNeighs == 3) grid.field[i][c] = 1;
			}
		}
		
	}
	
	public void printGrid()
	{
		for(int i=0; i<gridWidth; i++)
		{
			for (int c=0; c<gridHeight; c++)
			{
				System.out.print(grid.field[i][c]);
			}
			System.out.println();
		}
		System.out.println();
		System.out.println();
		System.out.println();
	}
	
	public static void main(String[] args)
	{
		GameOfLife gol = new GameOfLife(System.currentTimeMillis());
		gol.start();
		long steps = 0;
		gol.printGrid();
		while (steps < 1000)
		{
			if (!gol.schedule.step(gol))
				break;
			steps = gol.schedule.getSteps();
		}
		gol.printGrid();
		gol.finish();
		System.exit(0);
	}
	
}
