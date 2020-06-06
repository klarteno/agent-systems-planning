package org.agents.action;



public class PullAction extends Action {

	private Direction agentDir;
	private Direction boxDir;
	
	private int[] boxLocation;
	private int[] newBoxLocation;
	
	public PullAction(Direction agentDir, Direction boxDir, int[] location)
	{
		super(ActionType.PULL, location, Location.newLocation(location,agentDir));
		this.agentDir = agentDir;
		this.boxDir = boxDir;
		this.boxLocation 	=  Location.newLocation(getAgentLocation(),boxDir);
		this.newBoxLocation = getAgentLocation();
	}
	
	protected Direction getAgentDir()
	{
		return agentDir;
	}
	
	protected Direction getBoxDir()
	{
		return boxDir;
	}

	@Override
	public String toString()
	{
		return "Pull(" + this.agentDir.toString() + ","
				+ this.boxDir.toString() + ")";
	}
	
	@Override
	public Action getOpposite()
	{
		return new PushAction(this.getAgentDir().getOpposite(), this.getBoxDir(),
				Location.newLocation(getAgentLocation(),this.getAgentDir()));
	}
	
	@Override
	public boolean isOpposite(Action action) {
		if (action instanceof PushAction)
		{
			PushAction other = (PushAction) action;
			return Direction.isOpposite(this.getAgentDir(), other.getAgentDir())
					&& this.getBoxDir().equals(other.getBoxDir());
		}
		else
		{
			return false;			
		}
	}
}
