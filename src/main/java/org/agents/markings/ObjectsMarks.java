package org.agents;

import java.io.Serializable;

public final class ObjectsMarks implements Serializable {
    enum BoxField {
        LETTER_MARK_INDEX,
        COLOR_MARK_INDEX,
        SOLVED_STATUS;
    };

    public enum Coordinates {
        TIME,//includes waiting time in a cell location
        ROW,// coordinate Y
        COLUMN;//corrdinate X

        public int getTime(int [] pos){
            assert pos.length == 3;
            return pos[ObjectsMarks.Coordinates.TIME.ordinal()];
        }

        public int getRow(int [] pos){
            assert pos.length == 3;
            return pos[ObjectsMarks.Coordinates.ROW.ordinal()];
        }

        public int getCol(int [] pos){
            assert pos.length == 3;
            return pos[ObjectsMarks.Coordinates.COLUMN.ordinal()];
        }

        public static void setTime(int[] pos, int value){
            assert pos.length == 3;
            pos[ObjectsMarks.Coordinates.TIME.ordinal()] = value;
        }

        public static void setRow(int[] pos, int value){
            assert pos.length == 3;
            pos[ObjectsMarks.Coordinates.ROW.ordinal()] = value;
        }

        public static void setCol(int[] pos, int value){
            assert pos.length == 3;
            pos[ObjectsMarks.Coordinates.COLUMN.ordinal()] = value;
        }

    };

    enum AgentField {
        NUMBER_MARK_INDEX,
        COLOR_MARK_INDEX,
        SOLVED_STATUS;
    };
}
