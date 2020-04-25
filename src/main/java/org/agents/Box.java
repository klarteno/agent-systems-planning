package org.agents;

import java.io.Serializable;

public final class Box implements Serializable {
    //TO DO decapsulation of a new data strucure like a primitive array
    //TO DO bits for every field
    //public static final int NOT_SOLVED = 0;
    //public static final int GOT_SOLVED = 1;
    //public static final int IN_USE = 2;

    public enum SolvedStatus {
        NOT_SOLVED,
        GOT_SOLVED,
        IN_USE
    }

    private final int[] box_object;
    private final int[] box_object_coordinates;
    private int[] box_goal_coordinates;


    public Box(char letter_mark, int color_mark) {
        this.box_object = new int[ObjectsMarks.BoxField.values().length];
        this.box_object[ObjectsMarks.BoxField.LETTER_MARK_INDEX.ordinal()]= Character.getNumericValue(letter_mark);
        this.box_object[ObjectsMarks.BoxField.COLOR_MARK_INDEX.ordinal()]= color_mark;
        this.box_object[ObjectsMarks.BoxField.SOLVED_STATUS.ordinal()] = SolvedStatus.NOT_SOLVED.ordinal();

        this.box_object_coordinates = new int[ObjectsMarks.CoordinatesField.values().length];
        this.box_object_coordinates[ObjectsMarks.CoordinatesField.ROW_POS.ordinal()]= -1;
        this.box_object_coordinates[ObjectsMarks.CoordinatesField.COLUMN_POS.ordinal()]= -1;
    }

    public void setGoalPosition(int goal_row, int goal_column) {
        this.box_goal_coordinates = new int[ObjectsMarks.CoordinatesField.values().length];
        this.box_goal_coordinates[ObjectsMarks.CoordinatesField.ROW_POS.ordinal()]= goal_row;
        this.box_goal_coordinates[ObjectsMarks.CoordinatesField.COLUMN_POS.ordinal()]= goal_column;
    }

    public int[] getGoalPosition() {
        return this.box_goal_coordinates;
    }

    public void setRowPosition(int pos){
        if(this.valid(pos))
            this.box_object_coordinates[ObjectsMarks.CoordinatesField.ROW_POS.ordinal()]= pos;
    }

    public void setColumnPosition(int pos){
        if(this.valid(pos))
            this.box_object_coordinates[ObjectsMarks.CoordinatesField.COLUMN_POS.ordinal()]= pos;
    }

    //if the box is solved already returns false
    public boolean setSolvedStatus(SolvedStatus solvedStatus){
        if(this.box_object[ObjectsMarks.BoxField.SOLVED_STATUS.ordinal()] != SolvedStatus.GOT_SOLVED.ordinal()){
            this.box_object[ObjectsMarks.BoxField.SOLVED_STATUS.ordinal()]= solvedStatus.ordinal();
            return true;
        }
            return false;
        //change also the coordinates to be equlal to the goal position ?????

    }

    public SolvedStatus getSolvedStatus(){
        return SolvedStatus.values()[this.box_object[ObjectsMarks.BoxField.SOLVED_STATUS.ordinal()]] ;
    }

    public int getRowPosition(){
           return this.box_object_coordinates[ObjectsMarks.CoordinatesField.ROW_POS.ordinal()];
    }

    public int getColumnPosition(){
            return this.box_object_coordinates[ObjectsMarks.CoordinatesField.COLUMN_POS.ordinal()];
    }

    public int[] getCoordinates(){
        return this.box_object_coordinates;
    }


    public int getLetterMark(){
        return this.box_object[ObjectsMarks.BoxField.LETTER_MARK_INDEX.ordinal()];
    }

    public int getColor(){
        return this.box_object[ObjectsMarks.BoxField.COLOR_MARK_INDEX.ordinal()];
    }


    private boolean valid(int pos) {
        return pos>=0;
    }

}
