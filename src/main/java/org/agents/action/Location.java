package org.agents.action;

public class Location {

	private final int[] cell_coordinates;

	public Location(int[] cell_coordinates)
	{
		this.cell_coordinates = cell_coordinates;
	}

	public int[] getCoordinates(){
		return this.cell_coordinates;
	}

	public int getY(){
		return this.cell_coordinates[0];
	}

	public int getX(){
		return this.cell_coordinates[1];
	}

	public static int[] newLocation(int[] cell_position, Direction next_direction)
	{
		switch (next_direction)
		{
			case NORTH	: return new int[]{cell_position[0] - 1, cell_position[1]};
			case SOUTH	: return new int[]{cell_position[0] + 1, cell_position[1]};
			case WEST	: return new int[]{cell_position[0] , cell_position[1]-1};
			case EAST	:  return new int[]{cell_position[0] , cell_position[1]+1};
			default		: throw new IllegalArgumentException("Not a valid direction");
		}
	}

}
