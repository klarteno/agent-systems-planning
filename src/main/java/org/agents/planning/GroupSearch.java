package org.agents.planning;

import org.agents.Agent;
import org.agents.Box;
import org.agents.MapFixedObjects;
import org.agents.Utils;
import org.agents.markings.Coordinates;
import org.agents.planning.conflicts.ConflictAvoidanceCheckingRules;
import org.agents.planning.conflicts.IllegalPathsStore;
import org.agents.planning.conflicts.dto.SimulationConflict;
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

        ArrayDeque<int[]> new_path_one;

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
    /*  * make coupled search from GroupIndependenceDetection to SearchEngineOD
        *   objects SearchTaskResult,SearchScheduled remain the same acroos the running of GroupIndependenceDetection
        *   only internals change
        * */

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
    public void runPreProceesOfGoallState2(SearchTaskResult searchTaskResult) {
        ArrayDeque<int[]> path_found = searchTaskResult.getPath();
        int[] agt_ids = searchTaskResult.getStartGroupAgents();
        
        int[] goal_coord = path_found.pop();
        int[] standard_state = null;
        boolean std_state_found = false;

        int number_movables = goal_coord.length / Coordinates.getLenght();
        ArrayDeque<int[]> intermediate_states = new ArrayDeque<>();

        intermediate_states.add(goal_coord);
        int[] cell_neighbour = new int[0];
        ArrayList<SimulationConflict> standard_to_conflicts = new ArrayList<>();
        while (!std_state_found){
            int[] next_coord = path_found.pop();

            for (int i = 0; i < number_movables; i++) {
                int row_coord = Coordinates.getRow(i, next_coord);
                int col_coord = Coordinates.getCol(i, next_coord);

                int row_goal = Coordinates.getRow(i, goal_coord);
                int col_goal = Coordinates.getCol(i, goal_coord);

                int mark_id = agt_ids[i];
                int[] pos_coordinates = next_coord;

                if (row_coord == row_goal && col_coord == col_goal){
                    intermediate_states.add(next_coord);
                    int coord_to_expand = i;

                    if(intermediate_states.size() > 0){
                        cell_neighbour = Coordinates.getCoordinatesAt(i, intermediate_states.peek());
                        StateSearchMAFactory.setConflictsStandardStateExpansion(coord_to_expand, pos_coordinates, cell_neighbour, standard_to_conflicts);

                    }else {

                        int[] position_to_expand = Coordinates.getCoordinatesAt(i, pos_coordinates);
                        LinkedList<int[]> neighbours = conflict_avoidance_checking_rules.getFreeNeighboursMA(mark_id, position_to_expand, new ArrayDeque<int[]>());
                        for(int [] cell_pos_neighbour : neighbours){
                            StateSearchMAFactory.setConflictsStandardStateExpansion(coord_to_expand, pos_coordinates, cell_pos_neighbour, standard_to_conflicts);
                        }
                    }
                    std_state_found = false;
                    break;
                }else {
                    std_state_found = true;
                }

                /* 1 2 3 4
                 intermediate_states: 9 8 7 6 5
                */

                int[] st = intermediate_states.peek();
                ArrayDeque<int []> conflicts_avoidance = new ArrayDeque<>();

                for ( SimulationConflict simulationConflict  : standard_to_conflicts ){
                    if(simulationConflict.getMarkedId() == mark_id ){
                        ArrayList<int[]> coord = simulationConflict.getCoordinatesToAvoid();
                        conflicts_avoidance.addAll(coord);
                    }
                }

                int[] position_to_expand = Coordinates.getCoordinatesAt(i, pos_coordinates);
                LinkedList<int[]> __neighbours = conflict_avoidance_checking_rules.getFreeNeighboursMA(mark_id, position_to_expand, conflicts_avoidance);


                if( __neighbours.contains(st) ) {

                    // keep st in intermediate_states


                }else{
                    ArrayDeque<int [][]> next_state_nodes = new ArrayDeque<>();

//replace st with one of the neighbour but only for one coordinate for one agent
                    int[] next_state_node;
                    for(int [] cell_pos : __neighbours){
                        next_state_node = Arrays.copyOf(pos_coordinates, pos_coordinates.length);
                        Coordinates.setCoordinateAtIndex(i, next_state_node, cell_pos);

                        int g_cost = 1 ;
                        int f_cost = 0;//close to goal
                        next_state_nodes.add(SearchMAState.createNew(next_state_node, g_cost, f_cost));
                    }

                }



            }
        }

        standard_state = path_found.peek();


       if(intermediate_states.size() > 0){
           int[] st = intermediate_states.pop();
           int[] pos_coordinates = st;

           //SearchEngineOD searchEngineOD = new SearchEngineOD();
               //LinkedList<int[]> path = searchEngineOD.expandFrom(standard_state, intermediate_states);

           //setConflictsStandardStateExpansion(coord_to_expand, pos_coordinates, cell_pos_neighbour, standard_to_conflicts);
       }else {
           throw new UnsupportedOperationException("ddddddd");
       }

    }


    //used for when the agent stops on the goal and the goal is a box
    public int[] runPreProceesOfGoallState(SearchTaskResult searchTaskResult, int[][] standard_state) {
        ArrayDeque<int[]> path_found = searchTaskResult.getPath();
        int[] agt_ids = searchTaskResult.getStartGroupAgents();

        int[] path_goal = path_found.pop();

        boolean std_state_found = false;

        int number_movables = path_goal.length / Coordinates.getLenght();
        ArrayDeque<int[]> intermediate_states = new ArrayDeque<>();

        intermediate_states.add(path_goal);

        int[] cell_neighbour = new int[0];
        while (!std_state_found){
            int[] next_coord = path_found.peek();

            for (int i = 0; i < number_movables; i++) {
                int row_coord = Coordinates.getRow(i, next_coord);
                int col_coord = Coordinates.getCol(i, next_coord);

                int row_goal = Coordinates.getRow(i, path_goal);
                int col_goal = Coordinates.getCol(i, path_goal);

                int mark_id = agt_ids[i];
                int[] pos_coordinates = next_coord;

                if (row_coord == row_goal && col_coord == col_goal){
                    intermediate_states.add(next_coord);
                    path_found.pop();
                    int coord_to_expand = i;

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

        ArrayList<SimulationConflict> standard_to_conflicts = new ArrayList<>();
        //add edge and vertex conflicts of agent to goal
        int[] pos_coordinates = path_found.peek();



        //StateSearchMAFactory. set something for total_group
        //make intermediate node with agents and boxes
        int[] boxes_pos = searchTaskResult.getGoals_coordinates_of_group();

        int[] int_pos = Utils.concatanateArr(path_goal, boxes_pos);
        int[] group_pos = Utils.concatanateArr(pos_coordinates, boxes_pos);

        standard_state[0] = Arrays.copyOf(group_pos, group_pos.length);


        for (int i = 0; i < path_goal.length / Coordinates.getLenght(); i++) {
            int time_step_agt = Coordinates.getTime(i, path_goal);
            int next_box_idx = path_goal.length / Coordinates.getLenght() + i;
            Coordinates.setTime(next_box_idx, group_pos,time_step_agt - 1);
            Coordinates.setTime(next_box_idx, int_pos,time_step_agt - 1);
        }

        int[][] total_group = searchTaskResult.getTotalGroup();
        int[] start_group = scheduling_group.getTotalGroup()[SearchScheduled.INDEX_OF_GROUP];
        StateSearchMAFactory.setGroup_marks_ids(start_group);
        StateSearchMAFactory.setNumber_of_movables(start_group.length);
        StateSearchMAFactory.setUpAgentsWithBoxesFromGroup();

        if(intermediate_states.size() > 0){
            int[] next_coord = intermediate_states.pop();
            for (int i = 0; i < number_movables; i++) {
                int row_coord = Coordinates.getRow(i, next_coord);
                int col_coord = Coordinates.getCol(i, next_coord);

                int row_pos = Coordinates.getRow(i, pos_coordinates);
                int col_pos = Coordinates.getCol(i, pos_coordinates);

                //index of intermediate position found
                int coord_to_expand = i;
                cell_neighbour = Coordinates.getCoordinatesAt(coord_to_expand, next_coord);
                StateSearchMAFactory.setConflictsStandardStateExpansion(coord_to_expand, group_pos, cell_neighbour, standard_to_conflicts);
            }
        }else {
            throw new UnsupportedOperationException("ddddddd");
        }

        ArrayDeque<int [][]> next_state_nodes = new ArrayDeque<>();

        for (int i = path_goal.length / Coordinates.getLenght(); i < agt_ids.length; i++) {
            if(standard_to_conflicts.size() > 0){
                ArrayDeque<int []> conflicts_avoidance = new ArrayDeque<>();

                int mark_id = agt_ids[i];
                for ( SimulationConflict simulationConflict  : standard_to_conflicts ){
                    int _mark_id = simulationConflict.getMarkedId();
                    if(_mark_id == mark_id ){
                        ArrayList<int[]> coord = simulationConflict.getCoordinatesToAvoid();
                        conflicts_avoidance.addAll(coord);
                    }
                }
                int[] position_to_expand = Coordinates.getCoordinatesAt(i, group_pos);

                LinkedList<int[]> __neighbours = conflict_avoidance_checking_rules.getFreeNeighboursMA(mark_id, position_to_expand, conflicts_avoidance);

                int[] next_state_node;
                for(int [] cell_pos : __neighbours){
                    next_state_node = Arrays.copyOf(int_pos, int_pos.length);
                    Coordinates.setCoordinateAtIndex(i, next_state_node, cell_pos);

                    int g_cost = 1 ;
                    int f_cost = 0;//close to goal
                    next_state_nodes.add(SearchMAState.createNew(next_state_node, g_cost, f_cost));
                }
            }
        }

        int[][] intermediate_node_costs = StateSearchMAFactory.createIntermediateNodeCosts();
        ArrayDeque<int[][]> final_nodes = new ArrayDeque<>();

        for (int[][] __pos : next_state_nodes){
            ArrayDeque<int[][]> __next_nodes = StateSearchMAFactory.expandIntermediateStateWithAgentsAndBoxes(__pos[0], intermediate_node_costs, standard_to_conflicts);
            final_nodes.addAll(__next_nodes);
        }

        assert final_nodes.peek() != null;
        //repeat until the result is standard node
        while (! StateSearchMAFactory.isStandardNode(final_nodes.peek()[0]) ){
            for (int[][] __pos : next_state_nodes){
                ArrayDeque<int[][]> __next_nodes = StateSearchMAFactory.expandIntermediateStateWithAgentsAndBoxes(__pos[0], intermediate_node_costs, standard_to_conflicts);
                final_nodes.addAll(__next_nodes);
            }
        }


        int[] state = final_nodes.peek()[0];
        StateSearchMAFactory.isStandardNode(state);
        path_found.push(path_goal);//put the end path state back in the whole path
    return state;

    }


}
