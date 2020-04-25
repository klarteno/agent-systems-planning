package org.agents.action;

public class Location {

	private int[] cell_coordinates;

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

	/**
	 * Computes a new Location based on current direction 
	 * and location.
	 * @param dir - Direction
	 * @param loc - Location
	 * @return The new Location.
	 */
	public static Location newLocation(Direction dir, Location loc)
	{        
		return loc.newLocation(dir);
	}
	
	public Location newLocation(Direction next_direction)
	{
	    switch (next_direction)
	    {
	    case NORTH	: return new Location(new int[]{this.getY() - 1, this.getX()});
	    case SOUTH	: return new Location(new int[]{this.getY() +1, this.getX()});
	    case WEST	: return new Location(new int[]{this.getY(), this.getX() - 1});
	    case EAST	: return new Location(new int[]{this.getY(), this.getX() + 1});
	    default		: throw new IllegalArgumentException("Not a valid direction");
	    }
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

	public Direction inDirection(Location other)
	{
			 if (this.getY() < other.getY()  && this.getX() < other.getX()) return Direction.SOUTH_EAST;
		else if (this.getY() > other.getY() && this.getX() < other.getX()) return Direction.NORTH_EAST;
		else if ( this.getY() < other.getY()  && this.getX() > other.getX()) return Direction.SOUTH_WEST;
		else if (this.getY() > other.getY() && this.getX() > other.getX()) return Direction.NORTH_WEST;
		else if (this.getX() < other.getX()) return Direction.EAST;
		else if (this.getX() > other.getX()) return Direction.WEST;
		else if (this.getY() < other.getY()) return Direction.SOUTH;
			 else if (this.getY() > other.getY()) return Direction.NORTH;
		else return null;
	}
}
