package org.agents.action;


public class SkipAction extends Action {

	public SkipAction(int[] location)
	{
		super(ActionType.SKIP, location, location);
	}
	
	@Override
	public String toString()
	{
		return "NoOp";
	}
	
	@Override
	public Action getOpposite() {
		return null;
	}
	
	@Override
	public boolean isOpposite(Action action) {
		return false;
	}
}
