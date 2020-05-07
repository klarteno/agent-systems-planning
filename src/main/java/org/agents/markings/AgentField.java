package org.agents.markings;

public enum AgentField {
       NUMBER_MARK_INDEX,
       COLOR_MARK_INDEX,
       SOLVED_STATUS;

    //TO DO decapsulation of a new data strucure like a primitive array
    //TO DO bits for every field
    public static int[] createAgentField(){
        return new int[AgentField.getLenght()];
    }

    public static int getNumber(int[] agent_object){
        assert agent_object.length == 3;
        return agent_object[AgentField.NUMBER_MARK_INDEX.ordinal()];
    }

    public static int getColor(int[] agent_object){
        assert agent_object.length == 3;
        return agent_object[AgentField.COLOR_MARK_INDEX.ordinal()];
    }

    public static SolvedStatus getSolved(int[] agent_object){
        assert agent_object.length == 3;
        return   SolvedStatus.get(agent_object[AgentField.SOLVED_STATUS.ordinal()]) ;
    }

    private static int getLenght()
    {
        return 3;//AgentField.values().length;
    }

    public static void setNumber(int[] agent_object, int value){
        assert agent_object.length == 3;
        agent_object[AgentField.NUMBER_MARK_INDEX.ordinal()]=value;
    }

    public static void setColor(int[] agent_object, int value){
        assert agent_object.length == 3;
        agent_object[AgentField.COLOR_MARK_INDEX.ordinal()]=value;
    }

    public static int setSolved(int[] agent_object, SolvedStatus isSolved){
        assert agent_object.length == 3;
        agent_object[AgentField.SOLVED_STATUS.ordinal()] = isSolved.ordinal();
        return agent_object[AgentField.SOLVED_STATUS.ordinal()];
    }
}
