package org.agents.planning;

import org.agents.Color;
import org.agents.MapFixedObjects;
import org.agents.markings.Coordinates;
import org.agents.planning.conflicts.ConflictAvoidanceTable;

import java.util.ArrayDeque;

public class ConflictAvoidanceCheckingRules {

    enum SearchState {
        NO_CHECK_CONFLICTS,
        CHECK_CONFLICTS;
    };
    //change this when the searching algorithm requires to check for conflicts
    public SearchState searchState = SearchState.NO_CHECK_CONFLICTS;

    //TO DO : extend to multiple ConflictAvoidanceTable
    private final ConflictAvoidanceTable conflictAvoidanceTable;

    public ConflictAvoidanceCheckingRules(ConflictAvoidanceTable conflictAvoidanceTable){
        this.conflictAvoidanceTable = conflictAvoidanceTable;
    }

    public ConflictAvoidanceTable getConflictsTable(){
        return this.conflictAvoidanceTable;
    }

    //gets the neighbours of the cell  and removes those that conflicts
    public ArrayDeque<int[]> getFreeNeighboursMA(int[] coordinates, int color_movable, int start_time_step){
        ArrayDeque<int[]> next_cells = new ArrayDeque<>();
        if(searchState == SearchState.NO_CHECK_CONFLICTS){
            return MapFixedObjects.getNeighbours(coordinates, color_movable);
        }
        else if(searchState == SearchState.CHECK_CONFLICTS){
            next_cells = MapFixedObjects.getNeighbours(coordinates, color_movable);
            int[] dir_wait = new int[]{Coordinates.getTime(0,coordinates)+1, coordinates[0], coordinates[1]};
            if (MapFixedObjects.isFreeCell(dir_wait, color_movable))
                next_cells.add(dir_wait);

            this.getCheckConflictAvoidanceTable(coordinates, start_time_step, next_cells);
            return next_cells;
        }
        return next_cells;
    }


    //gets the neighbours of the cell  and removes those that conflicts at the start_time_step it uses time delay to prune wait states
    public ArrayDeque<int[]> getFreeNeighbours(int[] coordinates, int color_movable, int start_time_step, int time_deadline_constraint){
        ArrayDeque<int[]> next_cells = new ArrayDeque<>();
        if(searchState == SearchState.NO_CHECK_CONFLICTS){
            return MapFixedObjects.getNeighbours(coordinates, color_movable);
        }
        else if(searchState == SearchState.CHECK_CONFLICTS){
            next_cells = MapFixedObjects.getNeighbours(coordinates, color_movable);

            if(time_deadline_constraint > 0 && Coordinates.getTime(0,coordinates) <= time_deadline_constraint){
                int[] dir_wait = new int[]{Coordinates.getTime(0,coordinates)+1, coordinates[0], coordinates[1]};
                if (MapFixedObjects.isFreeCell(dir_wait, color_movable))
                    next_cells.add(dir_wait);
            }

            this.getCheckConflictAvoidanceTable(coordinates, start_time_step, next_cells);
            return next_cells;
        }
        return next_cells;
    }

    private void getCheckConflictAvoidanceTable(int[] coordinates, int time_step, ArrayDeque<int[]> next_cells) {
        this.conflictAvoidanceTable.removeCellConflicts(coordinates, next_cells);
    }




}
