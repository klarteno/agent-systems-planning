package org.agents.action;



import java.util.LinkedList;
import java.util.List;
import java.util.Stack;

public abstract class Action {

	public enum ActionType {
		MOVE,
		PUSH, 
		PULL,
		SKIP
	}
	
	private ActionType type;
	private int[] agentLocation,
					 newAgentLocation;
	
	protected Action(ActionType type, int[] agentLocation, int[] newAgentLocation)
	{
		this.type 				= type;
		this.agentLocation  	= agentLocation;
		this.newAgentLocation 	= newAgentLocation;
	}

	public ActionType getType()
	{
		return this.type;
	}
	
	public int[] getAgentLocation()
	{
		return this.agentLocation;
	}
	
	public int[] getNewAgentLocation()
	{
		return this.newAgentLocation;
	}
	
	public static Stack<Action> EveryMove(int[] agentLocation, Action exceptAction)
	{
		Stack<Action> actions = new Stack<Action>();
		for (Direction dir : Direction.EVERY) 
			actions.add(new MoveAction(dir, agentLocation));
		
		if (exceptAction != null)
		{
			actions.remove(exceptAction.getOpposite());
		}
		return actions;
	}
	
	public static List<Action> EveryBox(int[]  agentLocation, Action exceptAction)
	{
		Stack<Action> actions = new Stack<Action>();
		for (Direction dir1 : Direction.EVERY)
			for (Direction dir2 : Direction.EVERY)
				if (!Direction.isOpposite(dir1, dir2))
					actions.add(new PushAction(dir1, dir2, agentLocation));
		
		for (Direction dir1 : Direction.EVERY)
			for (Direction dir2 : Direction.EVERY)
				if (dir1 != dir2)
					actions.add(new PullAction(dir1, dir2, agentLocation));
		
		if (exceptAction != null)
		{
			actions.remove(exceptAction.getOpposite());
		}
		return actions;
	}

	public abstract boolean isOpposite(Action action);
	
	public abstract Action getOpposite();
	
	@Override
	public abstract String toString();

}
