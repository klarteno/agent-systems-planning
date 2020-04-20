package org.agents;

public final class Box {
    //TO DO decapsulation of a new data strucure like a primitive array
    //TO DO bits for every field
    private final int not_solved = 0;
    private final int got_solved = 1;

    private final int[] box_object;
    private final int[] box_object_coordinates;

    public Box(char letter_mark, int color_mark) {
        this.box_object = new int[ObjectsMarks.BoxField.values().length];
        this.box_object[ObjectsMarks.BoxField.LETTER_MARK_INDEX.ordinal()]= Character.getNumericValue(letter_mark);
        this.box_object[ObjectsMarks.BoxField.COLOR_MARK_INDEX.ordinal()]= color_mark;
        this.box_object[ObjectsMarks.BoxField.SOLVED_STATUS.ordinal()]= not_solved;

        this.box_object_coordinates = new int[ObjectsMarks.CoordinatesField.values().length];
        this.box_object_coordinates[ObjectsMarks.CoordinatesField.ROW_POS.ordinal()]= -1;
        this.box_object_coordinates[ObjectsMarks.CoordinatesField.COLUMN_POS.ordinal()]= -1;
    }

    public void setGoalPosition(int goal_row, int goal_column) {
        this.box_object[ObjectsMarks.BoxField.GOAL_ROW.ordinal()]= Character.getNumericValue(goal_row);
        this.box_object[ObjectsMarks.BoxField.GOAL_COLUMN.ordinal()]= Character.getNumericValue(goal_column);
    }

    public int[] getGoalPosition() {
        return  new int[]{this.box_object[ObjectsMarks.BoxField.GOAL_ROW.ordinal()],this.box_object[ObjectsMarks.BoxField.GOAL_COLUMN.ordinal()]};
    }

        public void setRowPosition(int pos){
        if(this.valid(pos))
            this.box_object_coordinates[ObjectsMarks.CoordinatesField.ROW_POS.ordinal()]= pos;
    }

    public void setColumnPosition(int pos){
        if(this.valid(pos))
            this.box_object_coordinates[ObjectsMarks.CoordinatesField.COLUMN_POS.ordinal()]= pos;
    }

    public boolean setSolvedStatus(){
        //change also the coordinates to be equlal to the goal position ?????
        if(this.box_object[ObjectsMarks.BoxField.SOLVED_STATUS.ordinal()]== not_solved){
            this.box_object[ObjectsMarks.BoxField.SOLVED_STATUS.ordinal()]= got_solved;

            return true;
        }
            return false;
    }

    public int getRowPosition(int pos){
        if(this.valid(pos))
           return this.box_object_coordinates[ObjectsMarks.CoordinatesField.ROW_POS.ordinal()];
        throw new IndexOutOfBoundsException(-1);
    }

    public int getColumnPosition(int pos){
        if(this.valid(pos))
            return this.box_object_coordinates[ObjectsMarks.CoordinatesField.COLUMN_POS.ordinal()];
        throw new IndexOutOfBoundsException(-1);
    }

    public int getLetterMark(){
        return this.box_object[ObjectsMarks.BoxField.LETTER_MARK_INDEX.ordinal()];
    }

    private boolean valid(int pos) {
        return pos>=0;
    }

}
