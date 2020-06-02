package org.agents.planning.conflicts.dto;

import java.util.ArrayDeque;

public class CorridorConflict  extends SimulationConflict {
    //make bfs from the conflict backwards
    //or for both agents backwards
    public CorridorConflict(int mark_id_conflicted) {
        super(mark_id_conflicted);
    }

    @Override
    public int[] getMaxTimeDeadline() {
        return new int[0];
    }

    @Override
    public ArrayDeque<int[]> getCoordinatesToAvoid() {
        return null;
    }


}
