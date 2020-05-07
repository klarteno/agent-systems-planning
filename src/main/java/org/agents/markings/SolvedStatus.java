package org.agents.markings;

//public static final int NOT_SOLVED = 0;
//public static final int GOT_SOLVED = 1;
//public static final int IN_USE = 2;

public enum SolvedStatus {
    NOT_SOLVED,
    GOT_SOLVED,
    IN_USE;

    public static SolvedStatus get(int index) {
        assert index>-1 && index<3;

        switch (index) {
            case 0:  return NOT_SOLVED;
            case 1:  return GOT_SOLVED;
            default:  return IN_USE;
        }
    }

    public static int get(SolvedStatus gotSolved) {
        switch (gotSolved) {
            case NOT_SOLVED:  return 0;
            case GOT_SOLVED:  return 1;
            default:  return 2;
        }

    }
}
