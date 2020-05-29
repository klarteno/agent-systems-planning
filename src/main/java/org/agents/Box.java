package org.agents;

import org.agents.markings.BoxField;
import org.agents.markings.Coordinates;
import org.agents.markings.SolvedStatus;

import java.io.Serializable;

public final class Box implements Serializable {
    private static final int COST_G = 0;
    private static final int COST_HEURISTIC = 1;

    private final int[] box_object;
    private final int[] box_object_coordinates;
    private int[] box_goal_coordinates;

    private final int[] box_costs = new int[2];


    public Box(char letter_mark, int color_mark) {
        this.box_object = BoxField.createBoxField();
        BoxField.setLetter(this.box_object, Character.getNumericValue(letter_mark));
        BoxField.setColor(this.box_object, color_mark);
        BoxField.setSolved(this.box_object, SolvedStatus.NOT_SOLVED);

        this.box_object_coordinates = new int[Coordinates.getLenght()];
        Coordinates.setTime(this.box_object_coordinates,0);
        Coordinates.setRow(this.box_object_coordinates,-1);
        Coordinates.setCol(this.box_object_coordinates,-1);
    }

    public void setGoalPosition(int goal_row, int goal_column) {
        this.box_goal_coordinates = Coordinates.createCoordinates();
        Coordinates.setRow(this.box_goal_coordinates,goal_row);
        Coordinates.setCol(this.box_goal_coordinates,goal_column);

        this.box_costs[COST_HEURISTIC] = MapFixedObjects.getManhattenHeuristic(box_object_coordinates, box_goal_coordinates);
    }

    public int[] getGoalPosition() {
        return this.box_goal_coordinates;
    }

    public void setRowPosition(int pos){
        if(this.valid(pos))
            Coordinates.setRow(this.box_object_coordinates,pos);
    }

    public void setColumnPosition(int pos){
        if(this.valid(pos))
            Coordinates.setCol(this.box_object_coordinates,pos);
    }

    public int getCostHeuristic(){
        return this.box_costs[COST_HEURISTIC];
    }

    //if the box is solved already returns false
    public boolean setSolvedStatus(SolvedStatus solvedStatus){
        if(BoxField.getSolved(this.box_object)!= SolvedStatus.GOAL_FINAL_SOLVED){
            BoxField.setSolved(this.box_object, solvedStatus);
            return true;
        }
            return false;
        //change also the coordinates to be equlal to the goal position ?????
    }

    public SolvedStatus getSolvedStatus(){
        return BoxField.getSolved(this.box_object) ;
    }

    public int getRowPosition(){
           return Coordinates.getRow(this.box_object_coordinates);
    }

    public int getColumnPosition(){
        return Coordinates.getCol(this.box_object_coordinates);
    }

    public void setTimeStep(int step_time){
        Coordinates.setTime(this.box_object_coordinates,step_time);
    }

    public int getTimeStep(){
        return Coordinates.getTime(this.box_object_coordinates);
    }

    public int[] getCoordinates(){
        return this.box_object_coordinates;
    }

    public int getLetterMark(){
        return BoxField.getLetter(this.box_object);
    }

    public int getColor(){
        return BoxField.getColor(this.box_object);
    }

    private boolean valid(int pos) {
        return pos>=0;
    }

}
