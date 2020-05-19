package org.agents;

import datastructures.HungarianAlgorithmResizable;
import org.agents.markings.SolvedStatus;
import org.agents.planning.MovablesScheduling;
import org.agents.planning.schedulling.TaskScheduled;
import org.agents.searchengine.SearchEngineSA;

import java.io.Serializable;
import java.util.*;

public class DivideAndScheduleMovables {
    private final int[][] data_matrix;
    private final ArrayList<int[]> minimum_cost;
    HungarianAlgorithmResizable hungarian_algorithm;
    private ArrayList<Agent> agents_scheduled;
    private LinkedList<Box> boxes;

    //jobs are columns
    //costs are rows
    //agents and boxes should have the same color
    //we choose columns for agents
    //we choose rows for total path from agent to box to boxs goal
    public DivideAndScheduleMovables(int[][] dataMatrix) {
        this.data_matrix = dataMatrix;
        hungarian_algorithm = new HungarianAlgorithmResizable(dataMatrix);
        this.minimum_cost = hungarian_algorithm.getMinimumCost();
    }

    private void setMovables(LinkedList<Box> boxes) {
        this.boxes = boxes;
        this.agents_scheduled = new ArrayList<>(boxes.size());//at least the number of boxes
    }

    private void setUpBoxes(LinkedList<Box> boxes) {
        this.boxes = boxes;
        this.agents_scheduled = new ArrayList<>(boxes.size());//at least the number of boxes
     }

    private ArrayDeque<Agent> getAgentsScheduled2(LinkedList<Agent> agents){
        ArrayDeque<Agent> agents_to_schedule = new ArrayDeque<>();

        for (Box next_box: boxes){
            int next_box_color = next_box.getColor();

            for (Agent agent : agents) {
                if(agent.getColor() == next_box_color){
                    agent.setGoalPosition( next_box.getGoalPosition() );
                    agents_to_schedule.add(agent);
                    agents_scheduled.add(agent);
                }
            }
        }
        return agents_to_schedule;
    }

    public ArrayDeque<MovablesScheduling> getAgentsScheduled(LinkedList<Agent> agents_to_schedule){
        final int AGENTS = 0;
        final int BOXES = 0;

        HashMap<Integer, ArrayList<Agent>> groups_agents = new HashMap<>();
        HashMap<Integer, Serializable[][]> groups;

        for (Agent agent : agents_to_schedule) {
            if (groups_agents.containsKey(agent.getColor())) {
                groups_agents.get(agent.getColor()).add(agent);
             }else {
                ArrayList<Agent> arrayList = new ArrayList<>();
                arrayList.add(agent);
                groups_agents.put(agent.getColor(),arrayList);
            }
        }

        groups = new HashMap<>(groups_agents.size());
        for (Integer key_color : groups_agents.keySet()){
            ArrayList<Agent> _agents = groups_agents.get(key_color);
            Serializable[][] match_movables = new Serializable[2][];
            match_movables[AGENTS] = new Agent[_agents.size()];

            for (int i = 0; i < _agents.size(); i++) {
                match_movables[AGENTS][i] = _agents.get(i);
            }

            ArrayDeque<Box> boxes_by_color = MapFixedObjects.getBoxesByColor(key_color);
            match_movables[BOXES] = new Box[boxes_by_color.size()];
            int index_box = 0;
            while (!boxes_by_color.isEmpty())
                match_movables[BOXES][index_box++] = boxes_by_color.pop();

            groups.put(key_color, match_movables);
        }

        ArrayDeque<MovablesScheduling> agents_scheduled_opt = new ArrayDeque<>();
        //build matrix with heuristic costs
        for (Integer color : groups.keySet()){
            Serializable[][] _group = groups.get(color);
            int[][] heuristc_costs = new int[_group[BOXES].length][_group[AGENTS].length];
            Serializable[] agentss =_group[AGENTS];
            Serializable[] boxxxes = _group[BOXES];
            for (int box_no = 0; box_no < _group[BOXES].length ; box_no++) {
                Box box_to_next = (Box)boxxxes[box_no];
                for (int agent_i = 0; agent_i < _group[AGENTS].length ; agent_i++) {
                    assert _group[AGENTS][agent_i] instanceof Agent;
                    Agent agent = (Agent)agentss[agent_i];
                    int heuristic_value = box_to_next.getCostHeuristic() + SearchEngineSA.getHeuristic(agent.getCoordinates(), box_to_next.getCoordinates());
                    heuristc_costs[box_no][agent_i] = heuristic_value;
                }
            }

            MovablesScheduling movablesScheduling = new MovablesScheduling();

            if(_group[AGENTS].length > 1) {
                this.hungarian_algorithm = new HungarianAlgorithmResizable(heuristc_costs);
                ArrayList<int[]> values_resulted = this.hungarian_algorithm.getMinimumCost();
                for(int[] pair : values_resulted){
                    int agent_index = pair[0];
                    int box_index = pair[1];
                    Agent agent_opt = (Agent) agentss[agent_index];
                    Box box_opt = (Box) boxxxes[box_index];

                    movablesScheduling.setUpPair(agent_opt, box_opt);
                    agents_to_schedule.remove(agent_opt);
                }

            }else{
                int min_heuristic_value = heuristc_costs[0][0];
                int box_no = 0;
                for (box_no = 0; box_no < heuristc_costs.length ; box_no++) {
                    if (min_heuristic_value > heuristc_costs[box_no][0]){
                        min_heuristic_value = heuristc_costs[box_no][0];
                    }
                }

                Agent agent_opt = (Agent) agentss[0];
                Box box_opt = (Box) boxxxes[box_no];

                movablesScheduling.setUpPair(agent_opt, box_opt);
                agents_to_schedule.remove(agent_opt);

            }

            agents_scheduled_opt.add(movablesScheduling);
        }

        return agents_scheduled_opt;
    }



    private ArrayDeque<Box> getBoxesScheduled(){
        ArrayDeque<Box> boxes_to_schedule = new ArrayDeque<>();

        for (Agent agent : this.agents_scheduled) {
            if(agent.getSolvedStatus() == SolvedStatus.GOAL_STEP_SOLVED ){
                for(Box next_box : boxes){
                    if (Arrays.equals(agent.getGoalPosition(), next_box.getCoordinates())){
                        boxes_to_schedule.add(next_box);
                    }
                }
            }
        }

        return boxes_to_schedule;
    }



    public TaskScheduled getSearchResults(){
        HashMap<Integer,ArrayDeque<Integer>> agents_to_boxes = new HashMap<>();

        ArrayList<Integer> agents_solved_mark_ids = new ArrayList<>();
        Set<Map.Entry<Integer, ArrayDeque<Integer>>> round = agents_to_boxes.entrySet();
        ArrayList<Integer> boxes_solved_mark_ids = new ArrayList<>();

        for (Agent agent : this.agents_scheduled) {
            switch (agent.getSolvedStatus()) {
                case GOAL_STEP_SOLVED:
                    ArrayDeque<Integer> boxes_solved = new ArrayDeque<>() ;
                    for(Box next_box : boxes){
                        if (Arrays.equals(agent.getGoalPosition(), next_box.getCoordinates())){
                            boxes_solved.add(next_box.getLetterMark());
                        }
                    }
                    agents_to_boxes.put(agent.getNumberMark(), boxes_solved);
                    break;
                case GOAL_FINAL_SOLVED:
                    agents_solved_mark_ids.add(agent.getNumberMark());
                    break;
                case IN_USE:break;
                case NOT_SOLVED:break;
            }

        }

        TaskScheduled taskScheduled = new TaskScheduled();
        taskScheduled.addAggentsBoxes(agents_to_boxes);
        taskScheduled.addAgents(agents_solved_mark_ids);

        return taskScheduled;
    }

}




