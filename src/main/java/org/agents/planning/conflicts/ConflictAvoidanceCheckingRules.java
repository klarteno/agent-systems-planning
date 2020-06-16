package org.agents.planning.conflicts;

import org.agents.MapFixedObjects;
import org.agents.markings.Coordinates;
import org.agents.planning.conflicts.dto.SimulationConflict;
import org.agents.planning.schedulling.TaskScheduled;
import org.agents.planning.schedulling.TrackedGroups;

import java.util.*;

public final class ConflictAvoidanceCheckingRules {
    private final ArrayList<TaskScheduled> task_scheduled_list;

    private final IllegalPathsStore illegal_paths_store;

    private final ConflictAvoidanceTable conflict_avoidance_table;

    public enum SearchState {
        NO_CHECK_CONFLICTS,
        CHECK_TIME_DEADLINE,
        AVOID_PATH;
    }
    private SearchState search_state = SearchState.NO_CHECK_CONFLICTS;

    public ConflictAvoidanceCheckingRules(TrackedGroups trackedGroups) {
        this.conflict_avoidance_table = new ConflictAvoidanceTable(trackedGroups);

        this.illegal_paths_store = new IllegalPathsStore(this.conflict_avoidance_table );
        this.task_scheduled_list = new ArrayList<>();
    }

    //it is also resetting the TrackedGroups stored
    public void setTrackedGroups(TrackedGroups trackedGroups){
        this.conflict_avoidance_table.setTrackedGroups(trackedGroups);
    }

    //change this when the searching algorithm requires to check for conflicts
    public boolean setSearchState(SearchState searchState){
        if(this.search_state != searchState){
            this.search_state = searchState;
            return true;
        }
        return false;
    }

    public boolean clearTaskScheduledList(){
        if(this.task_scheduled_list.size()>0){
            this.task_scheduled_list.clear();
            return true;
        }
        return false;
    }

    public boolean replaceTaskScheduledFor(int[] group, ArrayDeque<int[]> new_path){
        boolean found = false;
        for (TaskScheduled taskScheduled : this.task_scheduled_list){
            if (taskScheduled.isTheSameGroupAs(group)){
                found = true;
                taskScheduled.replacePathsFor(group, new_path);
            }else {

            }
        }
        return found;
    }

    public boolean addPathsToTaskScheduledPahs(int[] group1, int[] group2, int[] group_marks_total, ArrayDeque<int[]> new_path_group){
        boolean found = false;
        boolean found1 = removeTaskScheduledFor(group1);
        boolean found2 = removeTaskScheduledFor(group2);
        found = found1 || found2;

        TaskScheduled taskScheduled = new TaskScheduled(group_marks_total, new_path_group);
        this.task_scheduled_list.add(taskScheduled);

        return found;
    }

    public boolean removeTaskScheduledFor(int[] group){
        boolean found = false;
        for (TaskScheduled taskScheduled : this.task_scheduled_list){
            if (taskScheduled.isTheSameGroupAs(group)){
                found = true;
                this.task_scheduled_list.remove(taskScheduled);
            }
        }
        return found;
    }

    //gets the paths from TaskScheduled for each movable and marks these in the path store
    public void addTaskScheduledPaths(TaskScheduled taskScheduled) {
        HashMap<Integer, ArrayDeque<int[]> > agents_path = taskScheduled.getAgentsToPaths();
        HashMap<Integer, ArrayDeque<int[]> > boxes_path = taskScheduled.getBoxesToPaths();

        for (Integer key : agents_path.keySet()){
            ArrayDeque<int[]> path = agents_path.get(key);
            this.conflict_avoidance_table.replaceMarkedPathFor(key, path);
        }

        for (Integer key : boxes_path.keySet()){
            ArrayDeque<int[]> path = boxes_path.get(key);
            this.conflict_avoidance_table.replaceMarkedPathFor(key, path);
        }

        this.task_scheduled_list.add(taskScheduled);
    }

    //returns valid final paths
    public ArrayList<TaskScheduled> getValidTasks() {
        //TO CHOOSE FROM BELLOW
        /* getAllPathsFromtableDubFromconflict_avoidance_table()*/
        /*
        ArrayList[] result = this.conflict_avoidance_table.getAllPathsFromtable();
        ArrayList<Set<Integer>> grouped_movables = result[0];
        ArrayList<int[][][]> grouped_paths = result[1];
        */
         return this.task_scheduled_list;
    }

    public ConflictAvoidanceTable getConflictsTable(){
        return this.conflict_avoidance_table;
    }

    public IllegalPathsStore getIllegalPathsStore() { return this.illegal_paths_store; }

    public boolean setNextConflictedMovables(int[] colided_ids){
        int[] ungrouped_movables = this.conflict_avoidance_table.getAllUnGroupedIDs();
        int index = 0;
        int prev_marked;
        int next_marked;

        while (index != ungrouped_movables.length - 1){
            prev_marked = ungrouped_movables[index];
            int j = index;
            j++;
            for (int i = j; i < ungrouped_movables.length; i++) {
                if (i == index) continue;
                next_marked = ungrouped_movables[i];
                ArrayList<SimulationConflict> conflicts = this.illegal_paths_store.getConflicts(new int[prev_marked], new int[next_marked]);
                if (conflicts.size() > 0){
                    colided_ids[0] = prev_marked;
                    colided_ids[1] = next_marked;
                    return true;
                }
            }
            index++;
        }
        return false;
    }

    public void setIllegalPathsOfGroup(int[] start_group, int[] conflicting_group, int[][][] conflicting_paths) {
        ArrayList<SimulationConflict> paths_conflicts = this.getIllegalPathsStore().getConflicts(start_group, conflicting_group);

        int[][][] start_group_paths = this.getConflictsTable().getMarkedPaths(start_group);

        if (paths_conflicts.size()>0){
            IllegalPath illegalPath = new IllegalPath(start_group, conflicting_group, paths_conflicts);
            illegalPath.addPaths(start_group_paths, conflicting_paths);
            this.getIllegalPathsStore().addIlegalPath(illegalPath);
        }
    }

    //gets the neighbours of the cell  and removes those that conflicts at the start_time_step it uses time delay to prune wait states
    public ArrayDeque<int[]> getFreeNeighboursSA(int[] coordinate, int mark_id){
        ArrayDeque<int[]> next_cells = new ArrayDeque<>();

        if(search_state == SearchState.NO_CHECK_CONFLICTS){
            return MapFixedObjects.getNeighbours(coordinate, mark_id);
        }
        else if(search_state == SearchState.CHECK_TIME_DEADLINE){
            int time_deadline_constraint = Coordinates.getTime(this.illegal_paths_store.getIllegalPath(mark_id).getDeadlineConstraint());
            int[] deadline_state = this.illegal_paths_store.getIllegalPath(mark_id).getDeadlineConstraint();
            int cost_t = getManhattenHeuristic(coordinate, deadline_state);
            time_deadline_constraint -= cost_t;
            int coordinate_time_step = Coordinates.getTime(coordinate);
            if(time_deadline_constraint > 0 && coordinate_time_step < time_deadline_constraint){
                int[] dir_wait = new int[]{Coordinates.getTime(coordinate)+1, Coordinates.getRow(coordinate), Coordinates.getCol(coordinate)};
                //if (MapFixedObjects.isFreeCell(dir_wait, mark_id))
                next_cells.add(dir_wait);
            }
            else {
                next_cells = MapFixedObjects.getNeighbours(coordinate, mark_id);
                ArrayList<int[][][]> illegal_paths = this.illegal_paths_store.checkIllegalPath(mark_id);
                this.conflict_avoidance_table.removeIllegalConflicts(coordinate, next_cells, illegal_paths);
            }
        }
        else if(search_state == SearchState.AVOID_PATH){
            IllegalPath paths = this.illegal_paths_store.getIllegalPath(mark_id);
            next_cells = MapFixedObjects.getNeighbours(coordinate, mark_id);

            ArrayList<int[][][]> illegal_paths = this.illegal_paths_store.checkIllegalPath(mark_id);
            this.conflict_avoidance_table.removeIllegalConflicts(coordinate, next_cells, illegal_paths);
        }
        return next_cells;
    }


    public LinkedList<int[]> getFreeNeighboursMA(int mark_id, int[] coordinate, ArrayDeque<int[]> conflicts_avoidance) {
        ArrayDeque<int[]> neighbours = getFreeNeighboursSA(coordinate, mark_id);
        LinkedList<int[]> dirs = discardConflictsMA(neighbours, conflicts_avoidance);

        return dirs;
    }


    private static LinkedList<int[]> discardConflictsMA(ArrayDeque<int[]> neighbours, ArrayDeque<int[]> conflicts_avoidance) {
        LinkedList<int[]> dirs = new LinkedList<>();

        while (!neighbours.isEmpty()){
            int[] cell = neighbours.pop();
            dirs.add(cell);
        }

        while (!conflicts_avoidance.isEmpty()){
            int[] cell = conflicts_avoidance.pop();
            for (int i = 0; i < dirs.size(); i++) {
                if (Arrays.equals(cell, dirs.get(i))){
                    int[] res = dirs.remove(i);
                }
            }
        }
        return dirs;
    }

    private void getCheckConflictAvoidanceTable(int[] coordinates, ArrayDeque<int[]> next_cells) {
        this.conflict_avoidance_table.removeCellConflicts(coordinates, next_cells);
    }

    private static int getManhattenHeuristic(int y, int x, int y_goal, int x_goal) {
        return Math.abs(y - y_goal) + Math.abs(x - x_goal);
    }

    private static int getManhattenHeuristic(int[] cell_coordinates, int[] goal_coordinates){
        return Math.abs(Coordinates.getRow(cell_coordinates) - Coordinates.getRow(goal_coordinates)) + Math.abs(Coordinates.getCol(cell_coordinates) - Coordinates.getCol(goal_coordinates))  ;
    }

    private int getConsistentHeuristic(int mark_id, int cost_time, int y, int x, int y_goal, int x_goal) {
        int[] deadline_state = this.illegal_paths_store.getIllegalPath(mark_id).getDeadlineConstraint();
        int time_deadline_constraint = Coordinates.getTime(deadline_state);

        int time_left = time_deadline_constraint - cost_time;
        int row_deadline_state = Coordinates.getRow(deadline_state);
        int col_deadline_state = Coordinates.getCol(deadline_state);

        if (time_left <= 0){
            return getManhattenHeuristic(y, x, y_goal,x_goal);
        } else{
            return time_left + getManhattenHeuristic(row_deadline_state, col_deadline_state, y_goal, x_goal);
        }

    }

    //cost_time(s) is total cost of the path until now
    private  int getConsistentHeuristic(int mark_id, int cost_time, int[] cell_coordinates, int[] goal_coordinates){
        int[] deadline_state = this.illegal_paths_store.getIllegalPath(mark_id).getDeadlineConstraint();
        int time_deadline_constraint = Coordinates.getTime(deadline_state);

        int time_left = time_deadline_constraint - cost_time;
        int row_deadline_state = Coordinates.getRow(deadline_state);
        int col_deadline_state = Coordinates.getCol(deadline_state);

        if (time_left <= 0){
            return getManhattenHeuristic(cell_coordinates, goal_coordinates);
        } else{
            return time_left + getManhattenHeuristic(row_deadline_state, col_deadline_state, Coordinates.getRow(goal_coordinates), Coordinates.getCol(goal_coordinates));
        }
    }

    public int getHeuristicOf(int mark_id, int cost_time, int y, int x, int y_goal, int x_goal) {
        switch (this.search_state){
            case CHECK_TIME_DEADLINE: return this.getConsistentHeuristic(mark_id, cost_time, y, x, y_goal, x_goal);
            case NO_CHECK_CONFLICTS: return getManhattenHeuristic(y, x, y_goal, x_goal);
            case AVOID_PATH: return getManhattenHeuristic(y, x, y_goal, x_goal);
        }
        return 0;

    }

    public int getHeuristicOf(int mark_id,  int[] cell_coordinate, int[] goal_coordinate){
        int cost_time = Coordinates.getTime(cell_coordinate);

        switch (this.search_state){
            case CHECK_TIME_DEADLINE: return this.getConsistentHeuristic(mark_id, cost_time, cell_coordinate, goal_coordinate);
            case NO_CHECK_CONFLICTS: return getManhattenHeuristic(cell_coordinate, goal_coordinate);
            case AVOID_PATH: return getManhattenHeuristic(cell_coordinate, goal_coordinate);
        }
        return 0;
    }

    public int getCostTimeCoordinate(int mark_id, int[] cell_coordinate) {
        switch (this.search_state){
            case CHECK_TIME_DEADLINE:
                int[] deadline_coordinate = this.illegal_paths_store.getIllegalPath(mark_id).getDeadlineConstraint();
                int time_deadline_constraint = Coordinates.getTime(deadline_coordinate);
                int time_left = time_deadline_constraint - getManhattenHeuristic(cell_coordinate, deadline_coordinate);

                return time_left;

            case NO_CHECK_CONFLICTS: return -1;
            case AVOID_PATH: return -1;
        }

        int[] deadline_coordinate = this.illegal_paths_store.getIllegalPath(mark_id).getDeadlineConstraint();
        int time_deadline_constraint = Coordinates.getTime(deadline_coordinate);
        int time_left = time_deadline_constraint - getManhattenHeuristic(cell_coordinate, deadline_coordinate);

        return time_left;
    }

}
