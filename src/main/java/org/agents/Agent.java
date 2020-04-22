package org.agents;


public final class Agent {
    //TO DO decapsulation of a new data strucure like a primitive array
    //TO DO bits for every field
    private final int not_solved = 0;
    private final int got_solved = 1;

    private final int[] agent_object;
    private final int[] agent_object_coordinates;

    public Agent(int number_mark, int color_mark) {
        this.agent_object = new int[ObjectsMarks.AgentField.values().length];
        this.agent_object[ObjectsMarks.AgentField.NUMBER_MARK_INDEX.ordinal()]= number_mark;  ;
        this.agent_object[ObjectsMarks.AgentField.COLOR_MARK_INDEX.ordinal()]= color_mark;  ;
        this.agent_object[ObjectsMarks.BoxField.SOLVED_STATUS.ordinal()]= not_solved;

        this.agent_object_coordinates = new int[ObjectsMarks.CoordinatesField.values().length];
        this.agent_object_coordinates[ObjectsMarks.CoordinatesField.ROW_POS.ordinal()]= -1;
        this.agent_object_coordinates[ObjectsMarks.CoordinatesField.COLUMN_POS.ordinal()]= -1;
    }

    public void setGoalPosition(int goal_row, int goal_column) {
        this.agent_object[ObjectsMarks.AgentField.GOAL_ROW.ordinal()]= goal_row;
        this.agent_object[ObjectsMarks.AgentField.GOAL_COLUMN.ordinal()]= goal_column;
    }

    public int[] getGoalPosition() {
        return  new int[]{ this.agent_object[ObjectsMarks.AgentField.GOAL_ROW.ordinal()],this.agent_object[ObjectsMarks.AgentField.GOAL_COLUMN.ordinal()]};
    }

    public void setRowPosition(int pos){
        if(this.valid(pos))
            this.agent_object_coordinates[ObjectsMarks.CoordinatesField.ROW_POS.ordinal()]= pos;
    }

    public void setColumnPosition(int pos){
        if(this.valid(pos))
            this.agent_object_coordinates[ObjectsMarks.CoordinatesField.COLUMN_POS.ordinal()]= pos;
    }

    public int getRowPosition(int pos){
        if(this.valid(pos))
            return this.agent_object_coordinates[ObjectsMarks.CoordinatesField.ROW_POS.ordinal()];
        throw new IndexOutOfBoundsException(-1);
    }

    public int getColumnPosition(int pos){
        if(this.valid(pos))
            return this.agent_object_coordinates[ObjectsMarks.CoordinatesField.COLUMN_POS.ordinal()];
        throw new IndexOutOfBoundsException(-1);
    }

    public boolean setSolvedStatus(){
        //change also the coordinates to be equlal to the goal position ?????
        if(this.agent_object[ObjectsMarks.AgentField.SOLVED_STATUS.ordinal()]== not_solved){
            this.agent_object[ObjectsMarks.AgentField.SOLVED_STATUS.ordinal()]= got_solved;

            return true;
        }
        return false;
    }

    public int getNumberMark(){
        return this.agent_object[ObjectsMarks.AgentField.NUMBER_MARK_INDEX.ordinal()];
    }

    private boolean valid(int pos) {
        return pos>=0;
    }

}