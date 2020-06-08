package org.agents.markings;

//public static final int NOT_SOLVED = 0;
//public static final int GOAL_FINAL_SOLVED = 1;
//public static final int GOAL_STEP_SOLVED = 2;
//public static final int IN_USE = 3;
//public static final int GOT_SOLVED = 1;

public enum SolvedStatus {
    NOT_SOLVED,
    GOAL_FINAL_SOLVED,
    GOAL_STEP_SOLVED,
    IN_USE;

    public static SolvedStatus get(int index) {
        assert index > -1 && index < 4;

        switch (index) {
            case 0:  return NOT_SOLVED;
            case 1:  return GOAL_FINAL_SOLVED;
            case 2:  return GOAL_STEP_SOLVED;
            case 3:  return IN_USE;
            default:  return IN_USE;
        }
    }

    public static int get(SolvedStatus gotSolved) {
        switch (gotSolved) {
            case NOT_SOLVED:  return 0;
            case GOAL_FINAL_SOLVED:  return 1;
            case GOAL_STEP_SOLVED:  return 2;
            case IN_USE:  return 3;
            default:  return 3;
        }
    }
}
