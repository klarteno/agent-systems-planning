package org.agents.planning.conflicts;

import org.agents.markings.Coordinates;
import org.agents.planning.conflicts.dto.EdgeConflict;
import org.agents.planning.conflicts.dto.SimulationConflict;
import org.agents.planning.conflicts.dto.VertexConflict;

import java.util.ArrayList;
import java.util.LinkedList;

public final class IllegalPathsStore {
    LinkedList<IllegalPath> ilegalPaths;
    private final ConflictAvoidanceTable conflict_avoidance_table;

    public  IllegalPathsStore(ConflictAvoidanceTable conflictAvoidanceTable) {
        this.ilegalPaths = new LinkedList<>();
        this.conflict_avoidance_table = conflictAvoidanceTable;
    }

    public void addIlegalPath(IllegalPath illegalPath){
        ilegalPaths.add(illegalPath);
    }


    public boolean removeAllIlegalPaths(){
        if(ilegalPaths.size() > 0){
            ilegalPaths.clear();
            return true;
        }

        return false;
    }


    ArrayList<int[][][]> checkIllegalPath(int mark_id){
        IllegalPath group_paths;
        ArrayList<int[][][]> paths = new ArrayList<>();
        for (IllegalPath ilegalPath : ilegalPaths) {
            group_paths = ilegalPath;
            int[] group = group_paths.getStartGroup();
            for (int id : group) {
                if (id == mark_id) {
                    paths = group_paths.getPaths();
                    group_paths.setChecked(true);
                    //ilegalPaths.remove(i);
                }
            }
        }
/*
        IllegalPath illegal_path = this.getIllegalPath(mark_id);
        ArrayList<int[][][]> paths2 = illegal_path.getPaths();
        illegal_path.setChecked(true);
        ilegalPaths.remove(illegal_path);
*/
        return paths;
    }

    public IllegalPath getIllegalPath(int mark_id){
        IllegalPath illegal_path = null;
        for (IllegalPath ilegalPath : ilegalPaths) {
            int[] group = ilegalPath.getStartGroup();
            for (int value : group) {
                if (value == mark_id) {
                    illegal_path = ilegalPath;
                    break;
                }
            }
        }
        return illegal_path;
    }

    //to use
    //mark_ids gets stored in SimulationConflict
    public final ArrayList<SimulationConflict> getConflicts(int[] mark_ids, int[] groups){
        ArrayList<SimulationConflict> conflicts_set = new ArrayList<>();
        ArrayList<SimulationConflict> id_conflicts;

        if (groups.length == 0){
            return conflicts_set;
        }

        for (int mark_id : mark_ids) {
            id_conflicts = getAllConflicts(mark_id, groups);
            conflicts_set.addAll(id_conflicts);
        }

        return conflicts_set;
    }

    private ArrayList<SimulationConflict> getAllConflicts(int mark_id, int[] groups){
        //int[] groups = this.conflict_avoidance_table.getAllUnGroupedIDs();
        ArrayList<SimulationConflict> conflicts_set = new ArrayList<>();
        EdgeConflict edgeConflicts = new EdgeConflict(mark_id);
        VertexConflict vertexConflicts = new VertexConflict(mark_id);


        int[][][] groups_paths = this.conflict_avoidance_table.getMarkedPaths(groups);
        int[][][] paths_to_check_temp = this.conflict_avoidance_table.getMarkedPaths(new int[]{mark_id});
        int[][] paths_to_check = paths_to_check_temp[0];

        int index = 0;

        for(int id : groups){
            if(id == mark_id){
                index++;
                continue;
            }
            int[][] paths = groups_paths[index++];

            for (int row_i = 0; row_i < paths_to_check.length; row_i++) {
                for (int col_j = 0; col_j < paths_to_check[row_i].length; col_j++) {
                    boolean test_temp1 = paths_to_check[row_i][col_j] != ConflictAvoidanceTable.CELL_MARK1_TO_AVOID
                            && paths[row_i][col_j] != ConflictAvoidanceTable.CELL_MARK1_TO_AVOID;
                    boolean test_temp2 = paths_to_check[row_i][col_j] != ConflictAvoidanceTable.CELL_MARK2_TO_AVOID
                            && paths[row_i][col_j] != ConflictAvoidanceTable.CELL_MARK2_TO_AVOID;

                    if(test_temp1 && test_temp2){

                    int time_step = paths_to_check[row_i][col_j];
                    int time_step_to_check = paths[row_i][col_j];

                    //is cell position conflict
                    if(time_step == time_step_to_check){
                        vertexConflicts.addConflictedCell(id, Coordinates.createCoordinates(time_step, row_i, col_j));
                    }else {
                        /*
                       Edge CONFLICT:
                                6 -- 7 : AGENT1
                                7 -- 6 : AGENT2
                        Example of edge conflicts:
                                 5
                               1 6 7
                                -1
                                     -1
                                    8 7 6
                                     -1
                        * */
                        //check for conflicts of passing on  edges between cells
                        if (time_step - time_step_to_check == -1){
                            //check north
                            int time_step_north = paths_to_check[row_i - 1][col_j];
                            int time_step_to_check_north = paths[row_i - 1][col_j];
                            if (time_step_north - time_step_to_check_north == 1){
                                edgeConflicts.addConflictedEdge(id, Coordinates.createCoordinates(time_step, row_i, col_j),
                                        Coordinates.createCoordinates(time_step_north,row_i - 1, col_j));
                            }

                            //check south
                            int time_step_south = paths_to_check[row_i + 1][col_j];
                            int time_step_to_check_south = paths[row_i + 1][col_j];
                            if (time_step_south - time_step_to_check_south == 1){
                                edgeConflicts.addConflictedEdge(id, Coordinates.createCoordinates(time_step, row_i, col_j),
                                        Coordinates.createCoordinates(time_step_south, row_i + 1, col_j));

                            }

                            //check east
                            int time_step_east = paths_to_check[row_i][col_j - 1];
                            int time_step_to_check_east = paths[row_i][col_j - 1];
                            if (time_step_east - time_step_to_check_east == 1){
                                edgeConflicts.addConflictedEdge(id, Coordinates.createCoordinates(time_step, row_i, col_j),
                                        Coordinates.createCoordinates(time_step_east, row_i, col_j - 1));
                            }

                            //check west
                            int time_step_west = paths_to_check[row_i][col_j + 1];
                            int time_step_to_check_west = paths[row_i][col_j + 1];
                            if (time_step_west - time_step_to_check_west == 1){
                                edgeConflicts.addConflictedEdge(id, Coordinates.createCoordinates(time_step, row_i, col_j),
                                        Coordinates.createCoordinates(time_step_west, row_i, col_j + 1));
                            }
                        }
                    }
                }
            }
            }
        }

        if(edgeConflicts.getConflictedIds().size()>0){
            conflicts_set.add(edgeConflicts);
        }
        if(vertexConflicts.getConflictedIds().size()>0){
            conflicts_set.add(vertexConflicts);
        }

        return conflicts_set;
    }

}




