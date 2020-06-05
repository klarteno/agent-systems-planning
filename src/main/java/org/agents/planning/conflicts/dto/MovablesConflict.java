package org.agents.planning.conflicts.dto;

import java.util.ArrayList;

public class MovablesConflict extends SimulationConflict {
    private int agent_id;
    private int movable_id;


    public MovablesConflict(int mark_id_conflicted) {
        super(mark_id_conflicted);
    }

    @Override
    public int[] getMaxTimeDeadline() {
        return new int[0];
    }

    @Override
    public ArrayList<int[]> getCoordinatesToAvoid() {
        return null;
    }
}
