package org.agents.planning;

import org.agents.Agent;
import org.agents.Box;
import org.agents.MapFixedObjects;
import org.agents.planning.conflicts.ConflictAvoidanceCheckingRules;
import org.agents.planning.conflicts.IllegalPathsStore;
import org.agents.searchengine.SearchEngineOD;
import org.agents.searchengine.SearchEngineSA;
import org.agents.searchengine.StateSearchMAFactory;

import java.io.IOException;
import java.io.Serializable;
import java.util.ArrayDeque;

public final class GroupSearch {
    private final ConflictAvoidanceCheckingRules conflict_avoidance_checking_rules;

    public GroupSearch(ConflictAvoidanceCheckingRules conflictAvoidanceCheckingRules) {
        this.conflict_avoidance_checking_rules = conflictAvoidanceCheckingRules;
    }


    //concatanates group 1 with group 2 , works only for two groups
    public ArrayDeque<int[]> runGroupSearchMA(int[] start_group) throws IOException {
        return runGroupSearchMA(start_group, new int[0], new int[0][][]);
    }

    public ArrayDeque<int[]> runGroupSearch(int[] start_group, int[] conflicting_group, int[][][] conflicting_paths) throws IOException {
        //boolean isChanged = conflict_avoidance_checking_rules.setSearchState(ConflictAvoidanceCheckingRules.SearchState.CHECK_TIME_DEADLINE);
        boolean isChanged = conflict_avoidance_checking_rules.setSearchState(ConflictAvoidanceCheckingRules.SearchState.AVOID_PATH);

        ArrayDeque<int[]> new_path_one;
        if(start_group.length == 1){
            IllegalPathsStore illegalPathsStore = conflict_avoidance_checking_rules.getIllegalPathsStore();
            conflict_avoidance_checking_rules.setIllegalPathsOfGroup(start_group, conflicting_group, conflicting_paths);

            SearchEngineSA searchEngineSA = new SearchEngineSA(conflict_avoidance_checking_rules);

            int mark_id = start_group[0];
            Serializable next_movable = MapFixedObjects.getByMarkNo(mark_id);

            if (next_movable instanceof Agent) {
                searchEngineSA.runAstar(  (Agent)next_movable );
            }else if (next_movable instanceof Box) {
                searchEngineSA.runAstar( (Box)next_movable );
            }
            new_path_one = searchEngineSA.getPath();
            illegalPathsStore.removeAllIlegalPaths();
        }else{
            new_path_one = this.runGroupSearchMA(start_group, conflicting_group, conflicting_paths);
            //group_one and new_path_one have the same ordering of indexes
        }

        return new_path_one;
    }

    //gets the path for start_group and avoids colisions in group path; conflict_path
    //the path retturned is indexed in the same order as start_group
    public ArrayDeque<int[]> runGroupSearchMA(int[] start_group, int[] conflicting_group, int[][][] conflicting_paths) throws IOException {
        boolean isChanged = conflict_avoidance_checking_rules.setSearchState(ConflictAvoidanceCheckingRules.SearchState.AVOID_PATH);

        IllegalPathsStore illegalPathsStore = conflict_avoidance_checking_rules.getIllegalPathsStore();
        illegalPathsStore.removeAllIlegalPaths();
        conflict_avoidance_checking_rules.setIllegalPathsOfGroup(start_group, conflicting_group, conflicting_paths);


        SearchEngineOD searchEngineOD = new SearchEngineOD(start_group, this.conflict_avoidance_checking_rules, StateSearchMAFactory.SearchState.AGENTS_AND_BOXES);//StateSearchMAFactory.SearchState.AGENTS_ONLY

        int[] start_coordinates = searchEngineOD.getStartCoordinatesOfGroup();
        int[] goal_coordinates = searchEngineOD.getGoalsCoordinatesOfGroup();
        searchEngineOD.runOperatorDecomposition(start_coordinates, goal_coordinates);

        return searchEngineOD.getPath();
    }

    public void runGroupSearchMA2(int movable1_id, int movable2_id) {
        throw new UnsupportedOperationException("group sarch not implemented");
    }
}
