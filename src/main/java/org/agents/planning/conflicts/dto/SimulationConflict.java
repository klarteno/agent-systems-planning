package org.agents.planning.conflicts.dto;

import org.agents.markings.Coordinates;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Set;

public abstract class SimulationConflict {
    int mark_id_conflicted;

    int max_t_deadline;
    int[] max_coordinate_deadline;

    HashMap<Integer,int[]> mark_id_start_conflicts;
    Set<Integer> movable_mark_id_to_conflicted_ids;

    public SimulationConflict(int mark_id_conflicted){
        this.mark_id_conflicted = mark_id_conflicted;
        this.mark_id_start_conflicts = new HashMap<>();
        this.movable_mark_id_to_conflicted_ids = new HashSet<>();

        this.max_coordinate_deadline = Coordinates.createCoordinates();
    }

    public int getMarkedId(){
        return this.mark_id_conflicted;
    }

    public Set<Integer> getConflictedIds(){
        return this.movable_mark_id_to_conflicted_ids;
    }

    public abstract int[] getMaxTimeDeadline();
}
