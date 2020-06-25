package org.agents.searchengine;

import org.agents.Utils;
import org.agents.markings.Coordinates;
import org.agents.planning.conflicts.ConflictAvoidanceCheckingRules;
import org.agents.planning.conflicts.dto.SimulationConflict;
import org.agents.planning.schedulling.SearchScheduled;

import java.io.IOException;
import java.util.*;

public class SearchEngineOD {
    private ArrayDeque<int[]> path;
    private static PriorityQueue<int[][]> frontier;
    ArrayDeque<int[]> path_normal;

    public SearchEngineOD(SearchScheduled scheduling_group, ConflictAvoidanceCheckingRules conflictAvoidanceCheckingRules , StateSearchMAFactory.SearchState searchMultiAgentState){
        int[] start_coordinates;

        int[] goals_coordinates = scheduling_group.getGoals_coordinates();
        HashMap<Integer, ArrayList<int[]>> goals_neighbours = scheduling_group.getGoals_neighbours();
        int[] start_group = scheduling_group.getTotalGroup()[SearchScheduled.INDEX_OF_GROUP];

        StateSearchMAFactory.searchMultiAgentState = searchMultiAgentState;
        switch (searchMultiAgentState)
        {
            case AGENTS_ONLY:
                start_coordinates = scheduling_group.getStart_coordinates();
                break;
            case AGENTS_AND_BOXES:
                start_coordinates = scheduling_group.getStart_coordinates_agts_boxes();
                int[] index_boxes = scheduling_group.getIndexBoxes();
                StateSearchMAFactory.setIndexBoxes(index_boxes);
                break;
            default:
                throw new IllegalStateException("Unexpected value: " + searchMultiAgentState);
        }
        
        StateSearchMAFactory.setAvoidanceCheckingRules(conflictAvoidanceCheckingRules);

        StateSearchMAFactory.setStartGroup(start_group, start_coordinates, goals_coordinates, goals_neighbours);
        
        //make second option for comparator
        frontier = new PriorityQueue<int[][]>(5, Comparator.comparingInt(SearchMAState::getFCost));
        path_normal = new ArrayDeque<>();
    }


    public int[] getStartCoordinatesOfGroup(){
        return StateSearchMAFactory.getStartCoordinatesGroup();
    }

    public int[] getGoalsCoordinatesOfGroup(){
        return StateSearchMAFactory.getGoalsCoordinatesGroup();
    }

    public SearchTaskResult getPath(){
        SearchTaskResult searchTaskResult = new SearchTaskResult(this.path);
        searchTaskResult.setGroup(StateSearchMAFactory.getStartGroup());
        searchTaskResult.addStartCoordinates(this.getStartCoordinatesOfGroup());
        searchTaskResult.addGoalCoordinates(this.getGoalsCoordinatesOfGroup());

        return searchTaskResult;
    }

    public int getPathCost(){
       // assert this.path.size() > 0;
        return path.size();
    }

    public boolean isPathFound() {
        return this.path.size() > 0;
    }

    public static int getHeuristic(int[] cell_coordinates, int[] goal_coordinates){
        return Math.abs(Coordinates.getRow(cell_coordinates) - Coordinates.getRow(goal_coordinates)) + Math.abs(Coordinates.getCol(cell_coordinates) - Coordinates.getCol(goal_coordinates))  ;
    }

    public void runOperatorDecomposition() throws IOException {
        assert ( this.getStartCoordinatesOfGroup().length % Coordinates.getLenght() )== 0;
        assert this.getStartCoordinatesOfGroup().length/Coordinates.getLenght() > 1;

        String logFileName = "runOperatorDecomposition_file";
        Utils.logStartForFile(logFileName);

        HashMap<Integer, ArrayList<SimulationConflict>> standard_to_conflicts = new HashMap<>();
        HashMap<Integer, int[]> intermediate_came_from_standard = new HashMap<>();
        HashMap<Integer, int[]> standard_came_from_intermediate = new HashMap<>();
        HashMap<Integer, int[]> intermediate_came_from_intermediate = new HashMap<>();
        StateSearchMAFactory.createStatesCameFrom();

        ArrayDeque<int[][] > path_to_test = new ArrayDeque<>();
        ArrayDeque<int[]> path = new ArrayDeque<>();

        frontier.clear();
        //StateSearchMAFactory.createCostSoFar();
        StateSearchMAFactory.createClosedSet();

        int[][] next_state = StateSearchMAFactory.createStartState(this.getStartCoordinatesOfGroup(), 0);
        int[] prev_standard_state;
        frontier.add(next_state);
        //StateSearchMAFactory.putCostSoFar(next_state);
        //StateSearchMAFactory.mark_state_inqueue(next_state,true);

        StateSearchMAFactory.updateCameFromPrevCell2(next_state, StateSearchMAFactory.createDummyState());

        //init state with dummy variables
        int[][] current_state = null;
        int[][] prev_state = StateSearchMAFactory.createDummyState(); //StateSearchMAFactory.createDummyState()

        while(!frontier.isEmpty()){
            current_state = frontier.poll();
            Utils.logAppendToFile(logFileName, current_state, frontier.size());//

            if (StateSearchMAFactory.isStandardNode(SearchMAState.getStateCoordinates(current_state))){
                int prev_time = SearchMAState.getTime(0, SearchMAState.getStateCoordinates(prev_state));
                int current_time = SearchMAState.getTime(0, SearchMAState.getStateCoordinates(current_state));
                if(current_time - prev_time != 1  ){
                    SearchMAState.setTimeStep(current_state, prev_time + 1);
                }

                prev_state = current_state;
                //path.push(SearchMAState.getStateCoordinates(current_state));
                path_to_test.push(current_state);

                int[] state_to_test = SearchMAState.getStateCoordinates(current_state);

                if (StateSearchMAFactory.isGoal(SearchMAState.getStateCoordinates(current_state))){
                    //this.path = path;
                    path_normal.add(SearchMAState.getStateCoordinates(current_state));
                    int[] next_key = StateSearchMAFactory.getCameFrom(SearchMAState.getStateCoordinates(current_state));

                    int[] next_key2;
                    boolean removed;
                    while (!Arrays.equals(next_key, SearchMAState.getStateCoordinates(StateSearchMAFactory.getDummyState()))){
                        path_normal.add(next_key);
                        next_key = StateSearchMAFactory.getCameFrom(next_key);
                        //next_key2 = next_key;
                        //removed = StateSearchMAFactory.removeCameFrom(next_key2, next_key);
                        /*         to remove
                        if(Coordinates.getRow(0, next_key) == 3 && Coordinates.getCol(0, next_key) == 5  ){
                            start_to_add_to_del = true;
                            standard_states_expanded_to_del.add(next_key);
                        }*/
                    }
                    //path_normal.add(this.getStartCoordinatesOfGroup());

                    this.path = path_normal;

                    return;
                }

                StateSearchMAFactory.addToClosedSet(current_state);

                //when expanding a standard state the tree rooted at this standard node colects all the conflicts for this tree in standard_to_conflicts
                ArrayList<SimulationConflict> standard_conflicts;
                int pos_key = Arrays.hashCode(SearchMAState.getStateCoordinates(current_state));
                if(standard_to_conflicts.containsKey(pos_key)){
                    standard_conflicts = standard_to_conflicts.get(pos_key);
                    //Utils.logAppendToFile(logFileName, "standard_conflicts called line 162");
                    //Utils.logAppendToFile(logFileName, current_state,444);
                }else {
                    standard_conflicts = new ArrayList<>();
                }

                ArrayDeque<int[][]> next_intermediate_nodes = StateSearchMAFactory.expandStandardState(current_state, standard_conflicts);
                standard_to_conflicts.put(pos_key, standard_conflicts);

                 //assert StateSearchMAFactory.isIntermediateNode(next_intermediate_nodes);
                while(!next_intermediate_nodes.isEmpty()){
                    int[][] state = next_intermediate_nodes.pop();
                    StateSearchMAFactory.updateCameFromPrevCell(intermediate_came_from_standard, state, current_state);
                    frontier.add(state);
                    //StateSearchMAFactory.putCostSoFar(state);
                    //StateSearchMAFactory.mark_state_inqueue(state,true);
                }
            }else  if (StateSearchMAFactory.isIntermediateNode(SearchMAState.getStateCoordinates(current_state))){
                int _intermadiate_state_key = Arrays.hashCode(SearchMAState.getStateCoordinates(current_state));
                ArrayList<SimulationConflict> standard_node_conflicts = null;
                if(intermediate_came_from_intermediate.containsKey(_intermadiate_state_key)){
                    while(intermediate_came_from_intermediate.containsKey(_intermadiate_state_key)){
                        _intermadiate_state_key =  Arrays.hashCode(intermediate_came_from_intermediate.get(_intermadiate_state_key))  ;
                    }
                    int[] prev_standard_state__ = intermediate_came_from_standard.get(_intermadiate_state_key);
                    int prev_standard_state_key = Arrays.hashCode(prev_standard_state__);
                    standard_node_conflicts = standard_to_conflicts.get(prev_standard_state_key);
                }
                if(intermediate_came_from_standard.containsKey(_intermadiate_state_key)){
                    int[] prev_standard_state__ = intermediate_came_from_standard.get(_intermadiate_state_key);
                    //int prev_standard_state_key = Arrays.hashCode(prev_standard_state__);
                    standard_node_conflicts = standard_to_conflicts.get(Arrays.hashCode(prev_standard_state__));
                }
                assert standard_node_conflicts != null;

                ArrayDeque<int[][]> next_nodes = StateSearchMAFactory.expandIntermediateState(current_state, standard_node_conflicts);
                int ___key = _intermadiate_state_key;
                if (standard_node_conflicts.size() > 0){
                    standard_to_conflicts.put(___key, standard_node_conflicts);
                }

                for (int[][] state : next_nodes) {
                    int[] pos = SearchMAState.getStateCoordinates(state);
                    if (StateSearchMAFactory.isStandardNode(pos)) {
                        if (!StateSearchMAFactory.isInClosedSet(state)){
                            StateSearchMAFactory.updateCameFromPrevCell(standard_came_from_intermediate, state, current_state);
                            int _pos_key = Arrays.hashCode(pos);
                            int[] intermadiate_state = standard_came_from_intermediate.get(_pos_key);
                            int intermadiate_state_key = Arrays.hashCode(intermadiate_state);
                            int[] intermadiate_state2 = intermadiate_state;

                            while(intermediate_came_from_intermediate.containsKey(intermadiate_state_key)){
                                intermadiate_state2 = intermadiate_state;
                                intermadiate_state_key =  Arrays.hashCode(intermediate_came_from_intermediate.get(intermadiate_state_key)) ;
                                //intermediate_came_from_intermediate.remove(intermadiate_state2,intermadiate_state);
                            }
                            prev_standard_state = intermediate_came_from_standard .get(intermadiate_state_key);
                            //intermediate_came_from_standard.remove(intermadiate_state, prev_standard_state);
                            StateSearchMAFactory.updateCameFromPrevCell(state, prev_standard_state);

                            StateSearchMAFactory.updateCameFromPrevCell(state, prev_standard_state);
                                 //how to prune
                            frontier.add(state);
                                //StateSearchMAFactory.putCostSoFar(state);
                                //StateSearchMAFactory.mark_state_inqueue(state,true);
                        }
                    }else {
                         StateSearchMAFactory.updateCameFromPrevCell(intermediate_came_from_intermediate, state, current_state);
                        //how to store moves
                        // and heuristics
                        frontier.add(state);
                    }
                }
            }
        }
        //if the goal is not found return the empty path
        path.clear();
        this.path = path;
    }


}





