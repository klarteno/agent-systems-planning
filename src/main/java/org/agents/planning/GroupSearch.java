package org.agents.planning;

import org.agents.Agent;
import org.agents.Box;
import org.agents.MapFixedObjects;
import org.agents.Utils;
import org.agents.markings.Coordinates;
import org.agents.planning.conflicts.ConflictAvoidanceCheckingRules;
import org.agents.planning.conflicts.IllegalPathsStore;
import org.agents.planning.schedulling.SearchScheduled;
import org.agents.planning.schedulling.TrackedGroups;
import org.agents.searchengine.*;

import java.io.IOException;
import java.io.Serializable;
import java.util.*;


public final class GroupSearch {
    private final TrackedGroups root_tracked_group;
    private final ArrayDeque<TrackedGroups> in_process_tracked_groups;

    private final ConflictAvoidanceCheckingRules conflict_avoidance_checking_rules;
    private StateSearchMAFactory.SearchState search_state = StateSearchMAFactory.SearchState.AGENTS_AND_BOXES;
    private SearchTaskResult search_task_independence_detection;

    private SearchScheduled scheduling_group;

    public GroupSearch(TrackedGroups trackedGroups) {
        this.conflict_avoidance_checking_rules = new ConflictAvoidanceCheckingRules(trackedGroups);
        Set<Integer> agts = trackedGroups.getAgentsScheduledIds();
        Set<Integer> boxes = trackedGroups.getBoxesScheduledIds();
        //this.conflict_avoidance_checking_rules.setTrackedGroups(trackedGroups);
        this.root_tracked_group = trackedGroups;
        in_process_tracked_groups = new ArrayDeque<>();

        setUpIndependenceDetection();
    }

    public SearchTaskResult runAgentsSearchMA(SearchScheduled sched_group, int[] conflicting_group, int[][][] conflicting_paths) throws IOException {
        int[] start_group_agents = sched_group.getTotalGroup()[SearchScheduled.START_GROUP_AGENTS];
        int start_boxes_length = sched_group.getTotalGroup()[SearchScheduled.INDEX_OF_GROUP].length - start_group_agents.length;

        setSearchState(StateSearchMAFactory.SearchState.AGENTS_ONLY);
        setNextTracked(start_group_agents);
        this.scheduling_group = sched_group;

        SearchTaskResult searchTaskResult = getSearchMATaskResult(sched_group, conflicting_group, conflicting_paths);
        searchTaskResult.setTotalGroup(sched_group.getTotalGroup());
        searchTaskResult.setAgentstIdxsToBoxesIdxs(sched_group.getAgentstIdxsToBoxesIdxs());

        UUID uniqueID = UUID.randomUUID();
        sched_group.setUUID(uniqueID);
        searchTaskResult.setUUID(uniqueID);

        return searchTaskResult;
    }

    public SearchTaskResult runAgentsBoxesSearchMA(SearchScheduled sched_group, int[] conflicting_group, int[][][] conflicting_paths) throws IOException {
        int[] start_group = sched_group.getTotalGroup()[SearchScheduled.INDEX_OF_GROUP];

        setSearchState(StateSearchMAFactory.SearchState.AGENTS_AND_BOXES);
        setNextTracked(start_group);
        this.scheduling_group = sched_group;

        SearchTaskResult searchTaskResult = getSearchMATaskResult(sched_group, conflicting_group, conflicting_paths);
        searchTaskResult.setTotalGroup(sched_group.getTotalGroup());
        searchTaskResult.setAgentstIdxsToBoxesIdxs(sched_group.getAgentstIdxsToBoxesIdxs());

        UUID uniqueID = UUID.randomUUID();
        sched_group.setUUID(uniqueID);
        searchTaskResult.setUUID(uniqueID);

        return searchTaskResult;
    }

    private SearchTaskResult getSearchMATaskResult(SearchScheduled sched_group, int[] conflicting_group, int[][][] conflicting_paths ) throws IOException {
        boolean isChanged = conflict_avoidance_checking_rules.setSearchState(ConflictAvoidanceCheckingRules.SearchState.AVOID_PATH);

        IllegalPathsStore illegalPathsStore = conflict_avoidance_checking_rules.getIllegalPathsStore();
        illegalPathsStore.removeAllIlegalPaths();
        int[] start_group = sched_group.getTotalGroup()[SearchScheduled.INDEX_OF_GROUP];
        if (!(conflicting_group.length == 0)){
            conflict_avoidance_checking_rules.setIllegalPathsOfGroup(start_group, conflicting_group, conflicting_paths);
        }

        scheduling_group.setStartGroup(this.search_state);
         SearchEngineOD searchEngineOD = new SearchEngineOD(scheduling_group,
                this.conflict_avoidance_checking_rules, this.search_state);

        int[] start_coordinates = searchEngineOD.getStartCoordinatesOfGroup();
        int[] goals_coordinates = searchEngineOD.getGoalsCoordinatesOfGroup();

        searchEngineOD.runOperatorDecomposition();

        SearchTaskResult searchTaskResult = searchEngineOD.getPath();
        searchTaskResult.setTrackedGroup(this.in_process_tracked_groups.pop());

        return searchTaskResult;
    }

    private void setNextTracked(int[] start_group_agents) {
        Set<Integer> agents_to_schedule = new HashSet<>();
        for (int start_group_agent : start_group_agents) {
            agents_to_schedule.add(start_group_agent);
        }

        TrackedGroups trackedGroups;
        /*
        if (this.root_tracked_group.getAgentsScheduledIds().containsAll(agents_to_schedule)){
            trackedGroups = new TrackedGroups(agents_to_schedule, new HashSet<>());
            trackedGroups.initIdsIndexes(start_group_agents , new int[0]);
            this.conflict_avoidance_checking_rules.setTrackedGroups(trackedGroups);
            this.in_process_tracked_groups.add(trackedGroups);
        }else {
            throw new IllegalArgumentException("agent id unknown");
        }
        */
        trackedGroups = new TrackedGroups(agents_to_schedule, new HashSet<>());
        trackedGroups.initIdsIndexes(start_group_agents , new int[0]);
        this.conflict_avoidance_checking_rules.setTrackedGroups(trackedGroups);
        this.in_process_tracked_groups.add(trackedGroups);
    }

    //called by GroupIndependenceDetection
    public SearchTaskResult runGroupSearch(SearchScheduled sched_group, int[] conflicting_group, int[][][] conflicting_paths) throws IOException {
        //boolean isChanged = conflict_avoidance_checking_rules.setSearchState(ConflictAvoidanceCheckingRules.SearchState.CHECK_TIME_DEADLINE);
        boolean isChanged = conflict_avoidance_checking_rules.setSearchState(ConflictAvoidanceCheckingRules.SearchState.AVOID_PATH);

        int[] start_group =  sched_group.getTotalGroup()[SearchScheduled.INDEX_OF_GROUP];
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
            this.search_task_independence_detection = searchEngineSA.getPath();

            illegalPathsStore.removeAllIlegalPaths();
        }else{
            this.search_task_independence_detection = this.runAgentsBoxesSearchMA(sched_group, conflicting_group, conflicting_paths);
         }

        return this.search_task_independence_detection;
    }

    public SearchTaskResult runGroupIndependenceDetection(SearchScheduled sched_group) throws IOException {
        int[] conflicting_group = new int[0];
        int[][][] conflicting_paths = new int[0][][];
        return this.runGroupIndependenceDetection(sched_group, conflicting_group, conflicting_paths);
    }

    private SearchTaskResult runGroupIndependenceDetection(SearchScheduled sched_group, int[] conflicting_group, int[][][] conflicting_paths) throws IOException {
        int[] start_group = sched_group.getTotalGroup()[SearchScheduled.INDEX_OF_GROUP];
        int[] agt = sched_group.getTotalGroup()[SearchScheduled.INDEX_OF_AGENTS];
        int start_boxes_length = start_group.length - agt.length;
         //setNextTrackedAgents(start_group);//?????????

        boolean isChanged = this.conflict_avoidance_checking_rules.setSearchState(ConflictAvoidanceCheckingRules.SearchState.AVOID_PATH);//?????????

        IllegalPathsStore illegalPathsStore = conflict_avoidance_checking_rules.getIllegalPathsStore();
        illegalPathsStore.removeAllIlegalPaths();
        conflict_avoidance_checking_rules.setIllegalPathsOfGroup(start_group, conflicting_group, conflicting_paths);

        scheduling_group.setStartGroup( this.search_state);
        int[] start_coordinates = scheduling_group.getStart_coordinates();
        int[] goals_coordinates = scheduling_group.getGoals_coordinates();
        HashMap<Integer, ArrayList<int[]>> goals_neighbours = scheduling_group.getGoals_neighbours();
        SearchEngineOD searchEngineOD = new SearchEngineOD(sched_group,
                this.conflict_avoidance_checking_rules, this.search_state);

        start_coordinates = searchEngineOD.getStartCoordinatesOfGroup();
        goals_coordinates = searchEngineOD.getGoalsCoordinatesOfGroup();
        searchEngineOD.runOperatorDecomposition();

        //this.search_task_independence_detection.setTrackedGroup(this.in_process_tracked_groups.pop());
        this.search_task_independence_detection = searchEngineOD.getPath();
        this.search_task_independence_detection.setTotalGroup(sched_group.getTotalGroup());
        this.search_task_independence_detection.setAgentstIdxsToBoxesIdxs(sched_group.getAgentstIdxsToBoxesIdxs());

        return this.search_task_independence_detection;
    }

    //SearchTaskResult to be used only by the GroupIndependenceDetection algorithm
    private void setUpIndependenceDetection(){
        ArrayDeque<int[]> path_empty = new ArrayDeque<>();
        this.search_task_independence_detection = new SearchTaskResult(path_empty);
    }

    public ConflictAvoidanceCheckingRules getConflictAvoidanceCheckingRules(){ return  this.conflict_avoidance_checking_rules; }

    private void setSearchState(StateSearchMAFactory.SearchState searchState) {
        this.search_state = searchState;
    }

    //used for when the agent stops on the goal and the goal is a box
    public int[] runPreProceesOfGoallState(SearchTaskResult searchTaskResult, int[][] standard_state) {
        ArrayDeque<int[]> path_found = searchTaskResult.getPath();
        int[] group_ids = searchTaskResult.getStartGroupAgents();

        int[] path_goal = path_found.pop();
        boolean std_state_found = false;

        int number_movables = path_goal.length / Coordinates.getLenght();
        ArrayDeque<int[]> intermediate_states = new ArrayDeque<>();
        intermediate_states.add(path_goal);

        int[] cell_neighbour;
        while (!std_state_found){
            int[] next_coord = path_found.peek();

            for (int i = 0; i < number_movables; i++) {
                assert next_coord != null;
                int row_coord = Coordinates.getRow(i, next_coord);
                int col_coord = Coordinates.getCol(i, next_coord);

                int row_goal = Coordinates.getRow(i, path_goal);
                int col_goal = Coordinates.getCol(i, path_goal);

                if (row_coord == row_goal && col_coord == col_goal){
                    intermediate_states.add(next_coord);
                    path_found.pop();

                    std_state_found = false;
                    break;
                }else {
                    std_state_found = true;
                }
                /* 1 2 3 4
                 intermediate_states: 9 8 7 6 5
                */
            }
        }
        //add edge and vertex conflicts of agent to goal
        int[] pos_coordinates = path_found.peek();

        int[] boxes_pos = searchTaskResult.getGoals_coordinates_of_group();

        int[] int_pos = Utils.concatanateArr(path_goal, boxes_pos);
        int[] group_pos = Utils.concatanateArr(pos_coordinates, boxes_pos);//standard node state

        standard_state[0] = Arrays.copyOf(group_pos, group_pos.length);

        for (int i = 0; i < path_goal.length / Coordinates.getLenght(); i++) {
            int time_step_agt = Coordinates.getTime(i, path_goal);
            int next_box_idx = path_goal.length / Coordinates.getLenght() + i;
            Coordinates.setTime(next_box_idx, group_pos,time_step_agt - 1);
            Coordinates.setTime(next_box_idx, int_pos,time_step_agt - 1);
        }

        int[][] total_group = searchTaskResult.getTotalGroup();
        int[] start_group = scheduling_group.getTotalGroup()[SearchScheduled.INDEX_OF_GROUP];
        StateSearchMAFactory.getConflictsManager().clear();
        StateSearchMAFactory.getODNodeStructure().clear();

        StateSearchMAFactory.setGroup_marks_ids(start_group);
        StateSearchMAFactory.setNumber_of_movables(start_group.length);
        StateSearchMAFactory.setUpAgentsWithBoxesFromGroup();

        if(intermediate_states.size() > 0){
            int[] next_coord = intermediate_states.pop();
            for (int coord_to_expand = 0; coord_to_expand < number_movables; coord_to_expand++) {
                //index of intermediate position found
                cell_neighbour = Coordinates.getCoordinatesAt(coord_to_expand, next_coord);
                int[] pos__to_expand = new int[0];
                StateSearchMAFactory.od_conflicts.setConflictsNodeExpansion(coord_to_expand,pos__to_expand, group_pos, cell_neighbour);
            }
        }else {
            throw new UnsupportedOperationException("ddddddd");
        }

        ArrayDeque<int [][]> next_state_nodes = new ArrayDeque<>();

        for (int i = path_goal.length / Coordinates.getLenght(); i < group_ids.length; i++) {
            ArrayDeque<int []> conflicts_avoidance = StateSearchMAFactory.getConflictsManager().getConflictsAvoidance(i, group_pos);
            if(conflicts_avoidance.size() > 0){
                int mark_id = group_ids[i];
                int[] position_to_expand = Coordinates.getCoordinatesAt(i, group_pos);
                LinkedList<int[]> __neighbours = conflict_avoidance_checking_rules.getFreeNeighboursMA(mark_id, position_to_expand, conflicts_avoidance);

                int[] next_state_node;
                for(int [] cell_pos : __neighbours){
                    next_state_node = Arrays.copyOf(int_pos, int_pos.length);
                    Coordinates.setCoordinateAtIndex(i, next_state_node, cell_pos);

                    int g_cost = 1 ;
                    int f_cost = 0;//close to goal
                    next_state_nodes.add(SearchMAState.createNew(next_state_node, g_cost, f_cost));

                    StateSearchMAFactory.getODNodeStructure().updateIntermediateFromStandard(next_state_node, group_pos);
                }
            }
        }

        ArrayDeque<int[][]> final_nodes = new ArrayDeque<>();
        int[] index_boxes = searchTaskResult.getIndexBoxes();
        StateSearchMAFactory.setIndexBoxes(index_boxes);
        StateSearchMAFactory.heuristicMetricsSearch.initIntermediateNodeCosts();
        //int[][] intermediate_node_costs = StateSearchMAFactory.heuristicMetricsSearch.createIntermediateNodeCosts();

//add next_state_nodes to have root group_pos
        for (int[][] __pos : next_state_nodes){
            ArrayDeque<int[][]> __next_nodes = StateSearchMAFactory.expandIntermediateStateWithAgentsAndBoxes(__pos[0]);
            final_nodes.addAll(__next_nodes);
        }

        assert final_nodes.peek() != null;
        //repeat until the result is standard node
        while (! StateSearchMAFactory.isStandardNode(final_nodes.peek()[0]) ){
            for (int[][] __pos : next_state_nodes){
                ArrayDeque<int[][]> __next_nodes = StateSearchMAFactory.expandIntermediateStateWithAgentsAndBoxes(__pos[0]);
                final_nodes.addAll(__next_nodes);
            }
        }

        int[] state = final_nodes.peek()[0];
        StateSearchMAFactory.isStandardNode(state);
        path_found.push(path_goal);//put the end path state back in the whole path
    return state;
    }
}
