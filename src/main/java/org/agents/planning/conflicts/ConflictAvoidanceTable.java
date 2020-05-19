package org.agents.planning.conflicts;

import datastructures.DisjointSet;
import org.agents.MapFixedObjects;
import org.agents.markings.Coordinates;

import java.io.Serializable;
import java.util.*;

public final class ConflictAvoidanceTable implements Serializable {
    private final PathsStoreQuerying pathsStoreQuerying;
    private Set<Integer> ungrouped_movables;
    private DisjointSet group_set;//groups the movables objects

    //the conflict avoidance is used for the agents and the boxes used as input from MapFixedObjects
    //MapFixedObjects is used only static in PathsStoreQuerying
    public ConflictAvoidanceTable() {
        this.pathsStoreQuerying = new PathsStoreQuerying();
    }

//replaces all the agents and boxes store in this class and PathsStoreQuerying with movables_ids
    public void setUpTracked(Collection<? extends Integer> movables_ids){
        this.ungrouped_movables = new HashSet<>(movables_ids.size());
        this.ungrouped_movables.addAll(movables_ids);
        this.group_set = new DisjointSet(movables_ids.size());

        //takes as input the  set_ids of the markings of all agents and boxes, number_of_movables is number of boxes and agents needed to track
        this.pathsStoreQuerying.setUpTracked(movables_ids);
    }

    public Integer[] getAllUnGroupedIDs(){
        return this.ungrouped_movables.toArray(new Integer[0]);
    }

    public ArrayDeque<Integer> getGroupOf(int mark_id) {
        ArrayDeque<Integer> group = new ArrayDeque<>();
        group.add(mark_id);

        int id_index = pathsStoreQuerying.getIndexFor(mark_id);
        int next_index;
        for (Integer next_mark_id : this.ungrouped_movables) {
            next_index = pathsStoreQuerying.getIndexFor(next_mark_id);
            if (this.group_set.areInSameSet(id_index, next_index))
                group.add(next_mark_id);
        }
        return group;
    }

    public boolean isUnGrouped(int mark_id){
       return (this.getGroupOf(mark_id).size() == 1);
    }

    //merge two mark_ids by their indexes retrieved from paths store
    //and removes their paths stored because the åaths can not be valid after merging
    public void groupIDs(int mark_id1, int mark_id2){
        assert this.ungrouped_movables.contains(mark_id1);
        assert this.ungrouped_movables.contains(mark_id2);

        int index1 = pathsStoreQuerying.getIndexFor(mark_id1);
        pathsStoreQuerying.removePath(mark_id1);
        int index2 = pathsStoreQuerying.getIndexFor(mark_id2);
        pathsStoreQuerying.removePath(mark_id2);

        this.group_set.mergeSets(index1, index2);
    }

    //merge two groups of mark_ids by their indexes retrieved from paths store
    //and removes their paths stored because the åaths can not be valid after merging
    public int[][] groupIDs(int[] group_one, int[] group_two){
        pathsStoreQuerying.removePath(group_one);
        pathsStoreQuerying.removePath(group_two);

        int group = pathsStoreQuerying.getIndexFor(group_one[0]);
        assert this.group_set.getSizeOfSet(group) == group_one.length;
        for (int mark_id : group_two) {
             this.group_set.mergeSets(group, pathsStoreQuerying.getIndexFor(mark_id));
        }

        int[][] grouped_marks = new int[2][];
        grouped_marks[0] = group_one;
        grouped_marks[1] = group_two;

        return grouped_marks;
    }

//returns the first two conflicted found that are not in a group
    public boolean getNextConflictedMovables(int[] colided_ids){
        Integer[] ungrouped_movables = this.getAllUnGroupedIDs();
        int time_step = -1;
        int prev_marked = ungrouped_movables[0];
        int[][] prev_path = this.pathsStoreQuerying.getPathCloneFor(prev_marked);

        int next_marked;
        for (int i = 1; i < ungrouped_movables.length; i++) {
            next_marked = ungrouped_movables[i];
            int[][] next_path = this.pathsStoreQuerying.getPathCloneFor(next_marked);
            if(PathsStoreQuerying.isOverlap(prev_path, next_path)){
                colided_ids[0] = prev_marked;
                colided_ids[1] = next_marked;
                return true;
            }
            prev_marked = next_marked;
            prev_path = next_path;
        }
        return false;
    }

    public  int getPathLenght(int movable_id) {
        return this.pathsStoreQuerying.getPathLenght(movable_id);
    }

    public  int getPathLenght(int[] group__one) {
        int sum = 0;
        for (int i = 0; i < group__one.length; i++) {
            sum += this.pathsStoreQuerying.getPathLenght(i);
        }
        return sum;
    }

    //adds the path and removes the path given
    public void addMarkedPathAndPopped(ArrayDeque<int[]> path, int number_mark){
        int[] cell_loc;
        while (!path.isEmpty()){
            cell_loc = path.pop();
            pathsStoreQuerying.setCellLocationOf(number_mark, cell_loc);
        }
    }

    public void addMarkedPathsFor(int[][] group_marks_total, ArrayDeque<int[]> paths) {
        pathsStoreQuerying.setCellLocationOf(group_marks_total, paths);
    }

    public void replaceMarkedPathFor(int number_mark, ArrayDeque<int[]> path){
        pathsStoreQuerying.removePath(number_mark);
         for (int[] cell_location : path) {
             pathsStoreQuerying.setCellLocationOf(number_mark, cell_location);
        }
    }

    //adds the non-conflicting path to the path store ,  group_marks and group_paths should be ordered by the same index
    public void replaceMarkedPathFor(int[] group_marks, ArrayDeque<int[]> group_paths ){
        assert Objects.requireNonNull(group_paths.peek()).length/Coordinates.getLenght() == group_marks.length;

        pathsStoreQuerying.removePath(group_marks);

        int[] cell_locations;
        while(!group_paths.isEmpty()){
            cell_locations = group_paths.pop();
            pathsStoreQuerying.setCellLocationOf(group_marks, cell_locations);
        }
    }

    public boolean removeMarkedPath(int number_mark){
        return pathsStoreQuerying.removePath(number_mark);
    }

    /*
    public int removePathConflictsToDel(int start_time_step, Vector<int[]> path, ArrayList<int[]> next_conflicts){
        next_conflicts.clear();
        int[] next_cell_location;
        int y_loc;
        int x_loc;
        int time;
        int next_id;
        for (int id = 0; id < table_ids.keySet().size(); id++) {
            next_id = table_ids.get(id);
            time = start_time_step;
            for (int j = 0; j < path.size(); j++) {
                next_cell_location = path.get(j);
                y_loc = next_cell_location[0];
                x_loc = next_cell_location[1];
                if(table_for_paths[next_id][y_loc][x_loc] == time){
                    next_conflicts.add(path.get(j));
                    path.remove(j);
                }
                time++;
            }
            if (next_conflicts.size() > 0){
                //movable_conflicted[0] = id;
                return id;
            }
        }
        return -1;
    }*/


    //removes the elements from the input path that are in conflict with the conflict table avoidance
    //next_conflicts is the elemnts that are in conflict conform CAT
    public int removeConflictsInPath(int[] prev_cell_location, ArrayDeque<int[]> path, ArrayList<int[]> next_conflicts){
        next_conflicts.clear();
        int[] next_cell_location;
        int time;
        Iterator<int[]> path_iterator;

        for (int index = 0; index < pathsStoreQuerying.getNumberOfPaths(); index++) {
            path_iterator = path.iterator();
             while (path_iterator.hasNext()) {
                next_cell_location = path_iterator.next();
                if (isCellOrEdgeConflicts(next_cell_location, prev_cell_location, index))
                    next_conflicts.add(next_cell_location);

                prev_cell_location = next_cell_location;
            }
            if (next_conflicts.size() > 0){
                //movable_conflicted[0] = id;
                return index;//return the conflicted index whch is found in  tracked_marking_ids of agent or box
            }
        }
        return -1;
    }

//removes edge conflicts and cell conflicts
    //TO DO : add more rules depending on the colors between boxes and agents
    public void removeCellConflicts(int[] prev_cell_location, ArrayDeque<int[]> path){
        int[] cell_location;
         Iterator<int[]> path_iterator;

        for (int index = 0; index < pathsStoreQuerying.getNumberOfPaths(); index++) {
            path_iterator = path.iterator();
            while (path_iterator.hasNext()){
                cell_location = path_iterator.next();
                if (isCellOrEdgeConflicts(cell_location, prev_cell_location, index))
                    path_iterator.remove();//is it removing the present element??
            }

            if (path.size() == 0){
                //movable_conflicted[0] = id;
                return;
            }
        }
    }

    private boolean isCellOrEdgeConflicts(int[] cell_location, int[] prev_cell_location, int index) {
        //check cell location conflicts
        if(pathsStoreQuerying.getTimeStep(index, cell_location) ==  Coordinates.getTime(cell_location)){
            return true;
        }
        return getEdgeConflicts(prev_cell_location, cell_location, index);
    }

    //check edge between cell locations for conflicts
    private boolean getEdgeConflicts(int[] prev_cell_location, int[] cell_location, int index){
        return (pathsStoreQuerying.getTimeStep(index, prev_cell_location) ==  Coordinates.getTime(cell_location)) && (pathsStoreQuerying.getTimeStep(index, cell_location) ==  Coordinates.getTime(cell_location)-1);
    }

    public int getAllTrackedMovables(){
        int set_size = this.pathsStoreQuerying.getNumberOfPaths();

        throw new UnsupportedOperationException();

    }

//do not know what to check here ??
    //perhaps if we have two conflicting groups that were resolved with the conflict avoidance table
    //and then we add a path to a new group and the new group conflicts again with the other resolved group
    public boolean isNewConflict(int [] group_one, int [] group_two) {
        return true;
    }


    //return matrixes indexed by mark id from group_marks ,each matrix stores the time step for a coordinate for a movable
    public int[][][] getMarkedPaths(int[] group_marks) {
        return  this.pathsStoreQuerying.getPathsCloneForGroup(group_marks);
    }


}
