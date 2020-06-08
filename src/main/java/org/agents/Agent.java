package org.agents;

import org.agents.markings.AgentField;
import org.agents.markings.Coordinates;
import org.agents.markings.SolvedStatus;

import java.io.Serializable;

public final class Agent implements Serializable {
    private final int[] agent_object;
    private final int[] agent_object_coordinates;
    private int[] agent_goal_coordinates;

    public Agent(int number_mark, int color_mark) {
        this.agent_object = AgentField.createAgentField();

        AgentField.setNumber(this.agent_object, number_mark);
        AgentField.setColor(this.agent_object, color_mark);
        AgentField.setSolved(this.agent_object, SolvedStatus.NOT_SOLVED);

        this.agent_object_coordinates = new int[Coordinates.getLenght()];
        Coordinates.setTime(this.agent_object_coordinates,0);
        Coordinates.setRow(this.agent_object_coordinates,-1);
        Coordinates.setCol(this.agent_object_coordinates,-1);
    }

    public void setGoalPosition(int goal_row, int goal_column) {
        if (this.agent_goal_coordinates == null)
            this.agent_goal_coordinates = new int[Coordinates.getLenght()];

        Coordinates.setRow(this.agent_goal_coordinates, goal_row);
        Coordinates.setCol(this.agent_goal_coordinates, goal_column);
    }

    public void setGoalPosition(int[] goal) {
        this.setGoalPosition(Coordinates.getRow(goal), Coordinates.getCol(goal));
    }

    public boolean updatePositionCoordinates(){
        boolean is_changed = false;

        switch (AgentField.getSolved(this.agent_object) )
        {
            case GOAL_STEP_SOLVED:
            case GOAL_FINAL_SOLVED:
                int[] goal_pos = this.getGoalPosition();
                this.setCoordinatesPosition(goal_pos);

                is_changed = true;
                break;

            case IN_USE:break;
            case NOT_SOLVED: break;

            default: break;
        }
        return is_changed;
    }

    public void setTimePosition(int step_time){
        Coordinates.setTime(this.agent_object_coordinates, step_time);
    }

    public int getTimePosition(){
        return Coordinates.getTime(this.agent_object_coordinates);
    }

    public void setCoordinatesPosition(int row, int col){
        if(this.valid(row) &&  this.valid(col)){
            Coordinates.setRow(this.agent_object_coordinates, row);
            Coordinates.setCol(this.agent_object_coordinates, col);
        }
    }

    public void setCoordinatesPosition(int[] pos_coordinates){
        this.setCoordinatesPosition(Coordinates.getRow(pos_coordinates), Coordinates.getCol(pos_coordinates));
    }

    public int getRowPosition(){
        return Coordinates.getRow(this.agent_object_coordinates);
    }

    public int getColumnPosition(){
        return Coordinates.getCol(this.agent_object_coordinates);
    }

    public int[] getGoalPosition() {
        return  this.agent_goal_coordinates;
    }

    public int[] getCoordinates(){
        return this.agent_object_coordinates;
    }

    //TO DO can be moved to interface
    //is changing also the pposition coordinates: time,y,x to be equlal to the goal position coordinates
    public synchronized void setSolvedStatus(SolvedStatus stepSolved){
        AgentField.setSolved(this.agent_object, stepSolved);

        switch (stepSolved) {
            case GOAL_STEP_SOLVED:
            case GOAL_FINAL_SOLVED:
                this.updatePositionCoordinates();
                break;
            case IN_USE: break;
            case NOT_SOLVED:break;
        }
    }

    public synchronized SolvedStatus getSolvedStatus() {
        return AgentField.getSolved(this.agent_object) ;
    }

    public int getNumberMark(){
        return AgentField.getNumber(this.agent_object);
    }

    public int getColor(){
        return AgentField.getColor(this.agent_object);
    }

    private boolean valid(int pos) {
        return pos>=0;
    }
}
