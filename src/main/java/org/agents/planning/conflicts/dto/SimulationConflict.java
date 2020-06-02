package org.agents.planning.conflicts.dto;

import org.agents.markings.Coordinates;

import java.util.*;

public abstract class SimulationConflict {
    int movable_mark_id;

    int max_t_deadline;
    int[] max_coordinate_deadline;

    HashMap<Integer,int[]> mark_id_start_conflicts;
    Set<Integer> movable_mark_id_to_conflicted_ids;

    public SimulationConflict(int movable_mark_id){
        this.movable_mark_id = movable_mark_id;
        this.mark_id_start_conflicts = new HashMap<>();
        this.movable_mark_id_to_conflicted_ids = new HashSet<>();

        this.max_coordinate_deadline = Coordinates.createCoordinates();
    }

    public int getMarkedId(){
        return this.movable_mark_id;
    }

    public Set<Integer> getConflictedIds(){
        return this.movable_mark_id_to_conflicted_ids;
    }

    public abstract int[] getMaxTimeDeadline();

    public abstract ArrayDeque<int[]> getCoordinatesToAvoid();
}
