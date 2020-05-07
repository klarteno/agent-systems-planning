package org.agents.planning;

import datastructures.DisjointSet;
import org.agents.markings.Coordinates;

import java.util.*;

public final class ConflictAvoidanceTable {
    private final PathsStoreQuerying pathsStoreQuerying;
    //make it a union set
    private Set<Integer> ungrouped_movables;
    private DisjointSet group_set;

    //the conflict avoidance is used only for the agents and the boxes used as input
    public ConflictAvoidanceTable(PathsStoreQuerying pathsStoreQuerying) {
        this.pathsStoreQuerying = pathsStoreQuerying;

        Collection<? extends Integer> movables_ids = pathsStoreQuerying.getTrackedIds();
        this.setUpTracked(movables_ids);
    }

    public void setUpTracked(Collection<? extends Integer> movables_ids){
        this.ungrouped_movables = new HashSet<>(movables_ids.size());
        this.ungrouped_movables.addAll(movables_ids);
        this.group_set = new DisjointSet(movables_ids.size());
    }

    public Integer[] getAllUnGroupedIDs(){
        return this.ungrouped_movables.toArray(new Integer[0]);
    }

    public boolean isUnGrouped(int mark_id1){
       return this.ungrouped_movables.contains(mark_id1);

    }
    //merge two mark_ids by their indexes retrieved from paths store
    public void groupIDs(int mark_id1, int mark_id2){
        int index1 = pathsStoreQuerying.getIndexFor(mark_id1);
        int index2 = pathsStoreQuerying.getIndexFor(mark_id2);

        this.group_set.mergeSets(index1, index2);
        this.ungrouped_movables.remove(mark_id1);
        this.ungrouped_movables.remove(mark_id2);
    }


//returns the first two conflicted found that are not in a group
    public boolean getNextConflictedMovables(int[] colided_ids){
        Integer[] ungrouped_movables = this.getAllUnGroupedIDs();
        int time_step = -1;
        int prev_marked = ungrouped_movables[0];
        int[][] prev_path = this.pathsStoreQuerying.getPathFor(prev_marked);

        int next_marked;
        for (int i = 1; i < ungrouped_movables.length; i++) {
            next_marked = ungrouped_movables[i];
            int[][] next_path = this.pathsStoreQuerying.getPathFor(next_marked);
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

    //adds the path and removes the path given
    public void addMarkedPathAndPopped(ArrayDeque<int[]> path, int number_mark){
        int[] cell_loc;
        while (!path.isEmpty()){
            cell_loc = path.pop();
            pathsStoreQuerying.setCellLocationOf(cell_loc, number_mark, Coordinates.getTime(cell_loc));
        }
    }

    public void addMarkedPath(ArrayDeque<int[]> path, int number_mark){
         for (int[] cell_location : path) {
             pathsStoreQuerying.setCellLocationOf(cell_location, number_mark, Coordinates.getTime(cell_location));
        }
    }

    public boolean removeMarkedPath(int number_mark){
       return pathsStoreQuerying.removePath( number_mark);
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




}
