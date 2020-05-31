package org.agents.planning;

import org.agents.Agent;
import org.agents.Box;
import org.agents.MapFixedObjects;
import org.agents.planning.conflicts.ConflictAvoidanceCheckingRules;
import org.agents.planning.conflicts.IllegalPath;
import org.agents.planning.conflicts.IllegalPathsStore;
import org.agents.planning.conflicts.dto.SimulationConflict;
import org.agents.searchengine.SearchEngineOD;
import org.agents.searchengine.SearchEngineSA;

import java.io.Serializable;
import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Arrays;

public final class GroupSearch {
    private final ConflictAvoidanceCheckingRules conflict_avoidance_checking_rules;

    public GroupSearch(ConflictAvoidanceCheckingRules conflictAvoidanceCheckingRules) {
        this.conflict_avoidance_checking_rules = conflictAvoidanceCheckingRules;
    }

    //concatanates group 1 with group 2 , works only for two groups
    public ArrayDeque<int[]> runGroupSearchMA(int[][] start_group) {
        int[] groups = Arrays.copyOf(start_group[0], start_group[0].length + start_group[1].length);
        System.arraycopy(start_group[1], 0, groups, start_group[0].length, start_group[1].length);

        return runGroupSearchMA(groups, new int[0], new int[0][][]);
    }

    public ArrayDeque<int[]> runGroupSearch(int[] start_group, int[] conflicting_group, int[][][] conflicting_paths) {
        //boolean isChanged = conflict_avoidance_checking_rules.setSearchState(ConflictAvoidanceCheckingRules.SearchState.CHECK_TIME_DEADLINE);
        boolean isChanged = conflict_avoidance_checking_rules.setSearchState(ConflictAvoidanceCheckingRules.SearchState.AVOID_PATH);

        ArrayDeque<int[]> new_path_one;
        if(start_group.length == 1){

            IllegalPathsStore illegalPathsStore = conflict_avoidance_checking_rules.getIllegalPathsStore();
            setIllegalPathsOfGroup(start_group, conflicting_group, conflicting_paths, illegalPathsStore);

            SearchEngineSA searchEngineSA = new SearchEngineSA(conflict_avoidance_checking_rules);

            int mark_id = start_group[0];
            Serializable next_movable = MapFixedObjects.getByMarkNo(mark_id);

            if (next_movable instanceof Agent) {
                searchEngineSA.runAstar(  (Agent)next_movable );
            }else if (next_movable instanceof Box) {
                searchEngineSA.runAstar( (Box)next_movable );
            }
            new_path_one = searchEngineSA.getPath();

        }else{
            new_path_one = this.runGroupSearchMA(start_group, conflicting_group, conflicting_paths);
            //group_one and new_path_one have the same ordering of indexes
        }

        return new_path_one;
    }

    private void setIllegalPathsOfGroup(int[] start_group, int[] conflicting_group, int[][][] conflicting_paths, IllegalPathsStore illegalPathsStore) {
        ArrayList<SimulationConflict> paths_conflicts = illegalPathsStore.getConflicts(start_group, conflicting_group);
        int[][][] start_group_paths = conflict_avoidance_checking_rules.getConflictsTable().getMarkedPaths(start_group);

        if (paths_conflicts.size()>0){
            IllegalPath illegalPath = new IllegalPath(start_group, conflicting_group, paths_conflicts);
            illegalPath.addPaths(start_group_paths, conflicting_paths);
            illegalPathsStore.addIlegalPath(illegalPath);
        }
    }

    //gets the path for start_group and avoids colisions in group path; conflict_path
    //the path retturned is indexed in the same order as start_group
    public ArrayDeque<int[]> runGroupSearchMA(int[] start_group, int[] conflicting_group, int[][][] conflicting_paths) {
        boolean isChanged = conflict_avoidance_checking_rules.setSearchState(ConflictAvoidanceCheckingRules.SearchState.AVOID_PATH);

        IllegalPathsStore illegalPathsStore = conflict_avoidance_checking_rules.getIllegalPathsStore();
        illegalPathsStore.removeAllIlegalPaths();
        setIllegalPathsOfGroup(start_group, conflicting_group, conflicting_paths, illegalPathsStore);

        SearchEngineOD searchEngineOD = new SearchEngineOD(start_group, this.conflict_avoidance_checking_rules);

        int[] start_coordinates = searchEngineOD.getStartCoordinatesOfGroup();
        int[] goal_coordinates = searchEngineOD.getGoalsCoordinatesOfGroup();
        searchEngineOD.runOperatorDecomposition(start_coordinates, goal_coordinates, conflicting_paths);

        return searchEngineOD.getPath();
    }

    public void runGroupSearchMA2(int movable1_id, int movable2_id) {
        throw new UnsupportedOperationException("group sarch not implemented");
    }
}
