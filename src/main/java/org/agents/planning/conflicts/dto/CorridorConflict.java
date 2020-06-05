package org.agents.planning.conflicts.dto;

import java.util.ArrayList;

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
    public ArrayList<int[]> getCoordinatesToAvoid() {
        return null;
    }


}
