package org.agents;

import org.agents.markings.AgentField;
import org.agents.markings.Coordinates;
import org.agents.markings.SolvedStatus;

import java.io.Serializable;
import java.util.ArrayDeque;
import java.util.ArrayList;

public final class Agent implements Serializable {
    private final int[] markings_field;
    private final int[] coordinates;
    private int[] goal_stop_coordinates;
    private ArrayDeque<int[]> goals;
    private ArrayList<int[]> solved_goals;

    public Agent(int number_mark, int color_mark) {
        this.markings_field = AgentField.createAgentField();

        AgentField.setNumber(this.markings_field, number_mark);
        AgentField.setColor(this.markings_field, color_mark);
        AgentField.setSolved(this.markings_field, SolvedStatus.NOT_SOLVED);

        this.coordinates = new int[Coordinates.getLenght()];
        Coordinates.setTime(this.coordinates,0);
        Coordinates.setRow(this.coordinates,-1);
        Coordinates.setCol(this.coordinates,-1);
    }

    //sets one goal where th agent gets :like a final ending stop position
    public void setGoalStopPosition(int goal_row, int goal_column) {
        if (this.goal_stop_coordinates == null)
            this.goal_stop_coordinates = new int[Coordinates.getLenght()];

        Coordinates.setRow(this.goal_stop_coordinates, goal_row);
        Coordinates.setCol(this.goal_stop_coordinates, goal_column);
    }

    //sets one goal where th agent gets :like a final ending stop position
    public void setGoalStopPosition(int[] goal) {
        this.setGoalStopPosition(Coordinates.getRow(goal), Coordinates.getCol(goal));
    }

    public boolean updatePositionCoordinates(){
        boolean is_changed = false;

        switch (AgentField.getSolved(this.markings_field) )
        {
            case GOAL_STEP_SOLVED:
            case GOAL_FINAL_SOLVED:
                int[] goal_pos = this.getGoalStopPosition();
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
        Coordinates.setTime(this.coordinates, step_time);
    }

    public int getTimePosition(){
        return Coordinates.getTime(this.coordinates);
    }

    public void setCoordinatesPosition(int row, int col){
        if(this.valid(row) &&  this.valid(col)){
            Coordinates.setRow(this.coordinates, row);
            Coordinates.setCol(this.coordinates, col);
        }
    }

    public void setCoordinatesPosition(int[] pos_coordinates){
        this.setCoordinatesPosition(Coordinates.getRow(pos_coordinates), Coordinates.getCol(pos_coordinates));
    }

    public int getRowPosition(){
        return Coordinates.getRow(this.coordinates);
    }

    public int getColumnPosition(){
        return Coordinates.getCol(this.coordinates);
    }

    public int[] getGoalStopPosition() {
        return  this.goal_stop_coordinates;
    }

    public int[] getCoordinates(){
        return this.coordinates;
    }

    //TO DO can be moved to interface
    //is changing also the pposition coordinates: time,y,x to be equlal to the goal position coordinates
    public synchronized void setSolvedStatus(SolvedStatus stepSolved){
        AgentField.setSolved(this.markings_field, stepSolved);

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
        return AgentField.getSolved(this.markings_field) ;
    }

    public int getNumberMark(){
        return AgentField.getNumber(this.markings_field);
    }

    public int getColor(){
        return AgentField.getColor(this.markings_field);
    }

    private boolean valid(int pos) {
        return pos >= 0;
    }

    public int[] getNextGoal(){
        int[] goal;
        if(this.goals.size() > 0){
            goal = this.goals.pop();
        }else {
            goal = Coordinates.getEmptyInstance(); //new int[0];
        }


        if (this.solved_goals == null) this.solved_goals = new ArrayList<>();
        this.solved_goals.add(goal);
        
        return goal;
    }
    
    public void addGoalPosition(int[] next_goal_cell) {
        if (this.goals == null) this.goals = new ArrayDeque<>();
        this.goals.add(next_goal_cell);
    }
}
