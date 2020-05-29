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

        Coordinates.setRow(this.agent_goal_coordinates,goal_row);
        Coordinates.setCol(this.agent_goal_coordinates,goal_column);
    }

    public void setGoalPosition(int[] goal) {
        if (this.agent_goal_coordinates == null)
            this.agent_goal_coordinates = new int[Coordinates.getLenght()];

        Coordinates.setRow(this.agent_goal_coordinates,Coordinates.getRow(goal));
        Coordinates.setCol(this.agent_goal_coordinates,Coordinates.getCol(goal));
    }

    //TO DO can be moved to interface
    public void setTimePosition(int step_time){
        Coordinates.setTime(this.agent_object_coordinates, step_time);
    }

    public int getTimePosition(){
        return Coordinates.getTime(this.agent_object_coordinates);
    }

    public void setRowPosition(int pos){
        if(this.valid(pos))
            Coordinates.setRow(this.agent_object_coordinates,pos);
    }

    public void setColumnPosition(int pos){
        if(this.valid(pos))
            Coordinates.setCol(this.agent_object_coordinates,pos);
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
    public void setSolvedStatus(SolvedStatus stepSolved){
        //change also the coordinates to be equlal to the goal position ?????
        AgentField.setSolved(this.agent_object, stepSolved);
    }
    public SolvedStatus getSolvedStatus() {
        return AgentField.getSolved(this.agent_object) ;
    }

    public int getNumberMark(){
        return AgentField.getNumber(this.agent_object);
    }

    //TO DO can be moved to interface
    public int getColor(){
        return AgentField.getColor(this.agent_object);
    }

    private boolean valid(int pos) {
        return pos>=0;
    }
}
