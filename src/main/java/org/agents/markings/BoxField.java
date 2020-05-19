package org.agents.markings;

public enum BoxField {
    LETTER_MARK_INDEX,
    COLOR_MARK_INDEX,
    SOLVED_STATUS;

    //TO DO decapsulation of a new data strucure like a primitive array
    //TO DO bits for every field
    public static int[] createBoxField(){
        return new int[BoxField.getLenght()];
    }

    public static int getLetter(int[] box_object){
        assert box_object.length == 3;
        return box_object[BoxField.LETTER_MARK_INDEX.ordinal()];
    }

    public static int getColor(int[] box_object){
        assert box_object.length == 3;
        return box_object[BoxField.COLOR_MARK_INDEX.ordinal()];
    }

    public static SolvedStatus getSolved(int[] box_object){
        assert box_object.length == 3;
        return   SolvedStatus.get(box_object[BoxField.SOLVED_STATUS.ordinal()]) ;
    }

    public static void setSolved(int[] box_object){
        assert box_object.length == 3;
        box_object[BoxField.SOLVED_STATUS.ordinal()] = SolvedStatus.GOAL_FINAL_SOLVED.ordinal();
    }

    private static int getLenght(){
        return 3;//BoxField.values().length;
    }


    public static void setLetter(int[] box_object,int value){
        assert box_object.length == 3;
        box_object[BoxField.LETTER_MARK_INDEX.ordinal()]=value;
    }

    public static void setColor(int[] box_object,int value){
        assert box_object.length == 3;
        box_object[BoxField.COLOR_MARK_INDEX.ordinal()]=value;
    }

    public static void setSolved(int[] box_object, SolvedStatus isSolved){
        assert box_object.length == 3;
        box_object[BoxField.SOLVED_STATUS.ordinal()] = isSolved.ordinal();
    }


}
