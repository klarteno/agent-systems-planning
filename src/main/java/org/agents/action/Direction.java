package org.agents.action;

import org.agents.markings.Coordinates;

public enum Direction {
	NORTH,
	WEST,
	EAST,
	SOUTH,
	NORTH_EAST,
	NORTH_WEST,
	SOUTH_EAST,
	SOUTH_WEST,
	WAIT;
	
	public static final Direction[] EVERY = {
			NORTH, SOUTH, WEST, EAST
	};



	public String toString()
	{
		switch (this)
		{
		case NORTH	: return "N";
		case SOUTH	: return "S";
		case WEST	: return "W";
		case EAST	: return "E";
		case WAIT	: return "NoOp";
		default: 	  throw new IllegalArgumentException("Invalid direction");
		}
		
	}

	public static boolean isOpposite(Direction d1, Direction d2)
	{
		return d1.ordinal() + d2.ordinal() == 3;
	}

	public Direction getOpposite()
	{
		switch (this)
		{
		case NORTH: return SOUTH;
		case SOUTH: return NORTH;
		case EAST:  return WEST;
		case WEST:  return EAST;
		default: 	throw new IllegalArgumentException("Invalid direction");
		}
		
	}

	public static Direction getDirectionsFrom(int[] from_cell, int[] destination_cell){
		return getDirectionsFrom(Coordinates.getRow(from_cell), Coordinates.getCol(from_cell),
				Coordinates.getRow(destination_cell), Coordinates.getCol(destination_cell));
	}

	public static Direction getDirectionsFrom(int from_row, int from_col, int to_row, int to_col) {
		int[] offset= new int[2];

		offset[0] = to_row - from_row;
		offset[1] = to_col - from_col;

		//to optimize with switch statement and bits shifting for negative numbers
		if (offset[0] == -1 && offset[1] == 0 ){
			return Direction.NORTH;
		}
		if (offset[0] == 1 && offset[1] == 0 ){
			return Direction.SOUTH;
		}
		if (offset[0] == 0 && offset[1] == 1 ){
			return Direction.EAST;
		}
		if (offset[0] == 0 && offset[1] == -1 ){
			return Direction.WEST;
		}

		return Direction.WAIT;//if (offset[0] == 0 && offset[1] == 0 )
	}



	public static Direction[] getDirectionsFromOf(int[] from_cell_locations, int[] destination_cell_locations, boolean all_coordinates ){
		assert from_cell_locations.length > 3;
		assert destination_cell_locations.length > 3;
		assert from_cell_locations.length % 3 == 0;
		assert destination_cell_locations.length % 3 == 0;

		assert all_coordinates;

		int number_of_movables = from_cell_locations.length /Coordinates.getLenght();
		Direction[] directions = new Direction[number_of_movables];

		int[] from_cell = new int[Coordinates.getLenght()];
		int[] destination_cell = new int[Coordinates.getLenght()];

		Direction next_direction;
		int time_step;
		int row;
		int column;
		for (int index = 0; index < number_of_movables; index += 1) {
			time_step = index;
			row = time_step +1;
			column = time_step +1;

			Coordinates.setTime(from_cell, Coordinates.getTime(index, from_cell_locations));
			Coordinates.setTime(destination_cell, Coordinates.getTime(index, destination_cell_locations));

			Coordinates.setRow(from_cell, Coordinates.getRow(index, from_cell_locations));
			Coordinates.setRow(destination_cell, Coordinates.getRow(index, destination_cell_locations));

			Coordinates.setCol(from_cell, Coordinates.getCol(index, from_cell_locations));
			Coordinates.setCol(destination_cell, Coordinates.getCol(index, destination_cell_locations));

			directions[index] = getDirectionsFrom(from_cell, destination_cell);
		}

		return directions;
	}

	//get the array of directions with normal indexing and indexes from the coordiantes stored in the index_agents
	public static Direction[] getDirectionsFrom(int[] from_cell_locations, int[] destination_cell_locations, int[] index_agents ){
		assert from_cell_locations.length > 3;
		assert destination_cell_locations.length > 3;
		assert from_cell_locations.length % 3 == 0;
		assert destination_cell_locations.length % 3 == 0;

 		Direction[] directions = new Direction[index_agents.length];

		int[] from_cell = new int[Coordinates.getLenght()];
		int[] destination_cell = new int[Coordinates.getLenght()];

		int coord_index;
		for (int id_index = 0; id_index < index_agents.length; id_index += 1) {
			coord_index = index_agents[id_index];

			Coordinates.setTime(from_cell, Coordinates.getTime(coord_index, from_cell_locations));
			Coordinates.setTime(destination_cell, Coordinates.getTime(coord_index, destination_cell_locations));

			Coordinates.setRow(from_cell, Coordinates.getRow(coord_index, from_cell_locations));
			Coordinates.setRow(destination_cell, Coordinates.getRow(coord_index, destination_cell_locations));

			Coordinates.setCol(from_cell, Coordinates.getCol(coord_index, from_cell_locations));
			Coordinates.setCol(destination_cell, Coordinates.getCol(coord_index, destination_cell_locations));

			Direction __direction = getDirectionsFrom(from_cell, destination_cell);
			directions[id_index] =__direction;
		}
		return directions;
	}

	public void getNextCellFrom(int[] position_cell){
		switch (this)
		{
			case NORTH:
				position_cell[0] += -1;//y
				return;
			case SOUTH:
				position_cell[0] += 1;//y
				return;
			case EAST:
				position_cell[1] += 1;//x
				return;
			case WEST:
				position_cell[1] += -1;//x
				return;
			case WAIT:
				return;

			default: 	throw new IllegalArgumentException("Invalid direction");
		}
	}


}
