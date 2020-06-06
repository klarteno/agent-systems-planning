package org.agents.action;


public class MoveAction extends Action {

	private Direction direction;
	
	public MoveAction(Direction next_direction, int[] cell_position)
	{
		super(ActionType.MOVE, cell_position, Location.newLocation( cell_position, next_direction) );
		this.direction = direction;
	}

	@Override
	public String toString()
	{
		return "Move(" + this.direction.toString() + ")";
	}
	
	@Override
	public Action getOpposite() {
		return new MoveAction(this.direction.getOpposite(), Location.newLocation( this.getAgentLocation(), this.direction));
	}
	
	@Override
	public boolean isOpposite(Action action) {
		return action instanceof MoveAction && Direction.isOpposite(((MoveAction) action).direction, this.direction);
	}

}
