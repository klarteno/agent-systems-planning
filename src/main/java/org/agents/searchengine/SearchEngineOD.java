package org.agents.searchengine;

import org.agents.Utils;
import org.agents.markings.Coordinates;
import org.agents.planning.conflicts.ConflictAvoidanceCheckingRules;
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

    public void runOperatorDecomposition() throws IOException {
        assert ( this.getStartCoordinatesOfGroup().length % Coordinates.getLenght() )== 0;
        assert this.getStartCoordinatesOfGroup().length/Coordinates.getLenght() > 1;

        String logFileName = "runOperatorDecomposition_file";
        Utils.logStartForFile(logFileName);

        ODNodeStructure od_node_structure = StateSearchMAFactory.getODNodeStructure();

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
        int[][] prev_state = StateSearchMAFactory.createDummyState();

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
                path_to_test.push(current_state);

                if (StateSearchMAFactory.isGoal(SearchMAState.getStateCoordinates(current_state))){
                    path_normal.add(SearchMAState.getStateCoordinates(current_state));
                    int[] next_key = StateSearchMAFactory.getCameFrom(SearchMAState.getStateCoordinates(current_state));

                    while (!Arrays.equals(next_key, SearchMAState.getStateCoordinates(StateSearchMAFactory.getDummyState()))){
                        path_normal.add(next_key);
                        next_key = StateSearchMAFactory.getCameFrom(next_key);
                    }

                    this.path = path_normal;

                    return;
                }

                StateSearchMAFactory.addToClosedSet(current_state);
                ArrayDeque<int[][]> next_intermediate_nodes = StateSearchMAFactory.expandStandardState(current_state);

                while(!next_intermediate_nodes.isEmpty()){
                    int[][] state = next_intermediate_nodes.pop();

                    od_node_structure.updateIntermediateFromStandard(StateSearchMAFactory.getCellCoordinates(state), StateSearchMAFactory.getCellCoordinates(current_state));
                    frontier.add(state);
                    //StateSearchMAFactory.putCostSoFar(state);
                    //StateSearchMAFactory.mark_state_inqueue(state,true);
                }
            }else  if (StateSearchMAFactory.isIntermediateNode(SearchMAState.getStateCoordinates(current_state))){
                ArrayDeque<int[][]> next_nodes = StateSearchMAFactory.expandIntermediateState(current_state);

                for (int[][] state : next_nodes) {
                    int[] pos = SearchMAState.getStateCoordinates(state);
                    if (StateSearchMAFactory.isStandardNode(pos)) {
                        if (!StateSearchMAFactory.isInClosedSet(state)){
                            prev_standard_state = od_node_structure.updateStandardFromIntermediate(StateSearchMAFactory.getCellCoordinates(state), StateSearchMAFactory.getCellCoordinates(current_state));
                            StateSearchMAFactory.updateCameFromPrevCell(state, prev_standard_state);
                            frontier.add(state);
                                //StateSearchMAFactory.putCostSoFar(state);
                                //StateSearchMAFactory.mark_state_inqueue(state,true);
                        }
                    }else {
                        od_node_structure.updateIntermediateFromIntermediate(StateSearchMAFactory.getCellCoordinates(state), StateSearchMAFactory.getCellCoordinates(current_state));
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





