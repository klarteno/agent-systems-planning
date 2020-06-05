package org.agents.planning.constraints;

import org.agents.planning.conflicts.dto.SimulationConflict;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;

/*holds the constraint of PULL that the agent imposes on a neighbour box of the same colour
* edge constraint maps to PUSH actions
* */
public class PullConstraint extends SimulationConflict {
    private final int box_index;

    //the cell positions where the movable has to move
    HashMap<Integer,int[]> constraint_cells2;

    public PullConstraint(int boxIndex, int movable_mark_id) {
        super(movable_mark_id);
        this.box_index = boxIndex;
        constraint_cells2 = new HashMap<>();
    }

    public void addNextMoveCell(int[] pull_move_cell) {
        int hash_key = Arrays.hashCode(pull_move_cell);
        constraint_cells2.put(hash_key, pull_move_cell);
    }

    public int getBox_index(){
        return this.box_index;
    }

    //gets the cell positions where the movable has to move or expand
    public ArrayList<int[]> getConstraint_cells() {
        return new ArrayList<>(constraint_cells2.values());
    }

    @Override
    public int[] getMaxTimeDeadline() {
        throw new UnsupportedOperationException("PullConstraint not supported");
        //return new int[0];
    }

    @Override
    public ArrayList<int[]> getCoordinatesToAvoid() {
        return new ArrayList<>(constraint_cells2.values());
         //return null;//return where the movable was previouse
    }
}
