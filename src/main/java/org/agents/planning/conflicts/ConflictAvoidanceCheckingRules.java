package org.agents.planning.conflicts;

import org.agents.MapFixedObjects;
import org.agents.markings.Coordinates;
import org.agents.planning.schedulling.TaskScheduled;
import org.agents.planning.schedulling.TrackedGroups;

import java.util.ArrayDeque;
import java.util.HashMap;
import java.util.LinkedList;

public class ConflictAvoidanceCheckingRules {
    LinkedList<TaskScheduled> task_scheduled_list;
    //TO DO : extend to multiple ConflictAvoidanceTable
    private final ConflictAvoidanceTable conflict_avoidance_table;

    enum SearchState {
        NO_CHECK_CONFLICTS,
        CHECK_CONFLICTS;
    };
    public ConflictAvoidanceCheckingRules(TrackedGroups trackedGroups) {
        this.conflict_avoidance_table = new ConflictAvoidanceTable(trackedGroups);
        this.task_scheduled_list = new LinkedList<>();
    }

    public ConflictAvoidanceCheckingRules(TrackedGroups trackedGroups, SearchState searchState) {
        this.conflict_avoidance_table = new ConflictAvoidanceTable(trackedGroups);
        this.task_scheduled_list = new LinkedList<>();
        this.search_state = searchState;
    }


    //change this when the searching algorithm requires to check for conflicts
    public SearchState search_state = SearchState.NO_CHECK_CONFLICTS;

    public ConflictAvoidanceTable getConflictsTable(){
        return this.conflict_avoidance_table;
    }

    //gets the paths from TaskScheduled for each movable and marks these in the path store
    public void addTaskScheduledPaths(TaskScheduled taskScheduled) {
        HashMap<Integer, ArrayDeque<int[]> > agents_path = taskScheduled.getAgentsToPaths();
        HashMap<Integer, ArrayDeque<int[]> > boxes_path = taskScheduled.getBoxesToPaths();

        for (Integer key : agents_path.keySet()){
            ArrayDeque<int[]> path = agents_path.get(key);
            this.conflict_avoidance_table.replaceMarkedPathFor(key, agents_path.get(key));
        }

        for (Integer key : boxes_path.keySet()){
            ArrayDeque<int[]> path = boxes_path.get(key);
            this.conflict_avoidance_table.replaceMarkedPathFor(key, boxes_path.get(key));
        }
    }

    //returns valid final paths
    public TaskScheduled getNextValidTask() {
        return  this.task_scheduled_list.poll();
     }

    //gets the neighbours of the cell  and removes those that conflicts
    public ArrayDeque<int[]> getFreeNeighboursMA(int[] coordinates, int color_movable, int start_time_step){
        ArrayDeque<int[]> next_cells = new ArrayDeque<>();
        if(search_state == SearchState.NO_CHECK_CONFLICTS){
            return MapFixedObjects.getNeighbours(coordinates, color_movable);
        }
        else if(search_state == SearchState.CHECK_CONFLICTS){
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
        if(search_state == SearchState.NO_CHECK_CONFLICTS){
            return MapFixedObjects.getNeighbours(coordinates, color_movable);
        }
        else if(search_state == SearchState.CHECK_CONFLICTS){
            next_cells = MapFixedObjects.getNeighbours(coordinates, color_movable);

            if(time_deadline_constraint > 0 && Coordinates.getTime(coordinates) <= time_deadline_constraint){
                int[] dir_wait = new int[]{Coordinates.getTime(coordinates)+1, coordinates[0], coordinates[1]};
                if (MapFixedObjects.isFreeCell(dir_wait, color_movable))
                    next_cells.add(dir_wait);
            }

            this.getCheckConflictAvoidanceTable(coordinates, start_time_step, next_cells);
            return next_cells;
        }
        return next_cells;
    }

    private void getCheckConflictAvoidanceTable(int[] coordinates, int time_step, ArrayDeque<int[]> next_cells) {
        this.conflict_avoidance_table.removeCellConflicts(coordinates, next_cells);
    }




}
