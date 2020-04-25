package org.agents.action;


public class PushAction extends Action {
	
	private Direction agentDir;
	private Direction boxDir;
	
	private int[] boxLocation;
	private int[] newBoxLocation;
	
	public PushAction(Direction agentDir, Direction boxDir, int[] location)
	{
		super(ActionType.PUSH, location, Location.newLocation(location,agentDir));
		this.agentDir 		= agentDir;
		this.boxDir 		= boxDir;	
		this.boxLocation	= Location.newLocation(getAgentLocation(),agentDir);
		this.newBoxLocation =  Location.newLocation(getNewAgentLocation(),boxDir);
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
		return "Push(" + agentDir.toString() + "," 
				+ boxDir.toString() + ")";
	}
	
	@Override
	public Action getOpposite()
	{
		return new PullAction(this.agentDir.getOpposite(), this.getBoxDir(),
				Location.newLocation(this.getAgentLocation(),this.agentDir));
	}
	
	@Override
	public boolean isOpposite(Action action) {
		if (action instanceof PullAction)
		{
			PullAction other = (PullAction) action;
			return Direction.isOpposite(this.agentDir, other.getAgentDir())
					&& this.getBoxDir().equals(other.getBoxDir());
		}
		else
		{
			return false;			
		}
	}
}
