package org.agents.planning.schedulling;

import datastructures.HungarianAlgorithmResizable;
import org.agents.Agent;
import org.agents.Box;
import org.agents.MapFixedObjects;
import org.agents.markings.Coordinates;
import org.agents.markings.SolvedStatus;
import org.agents.searchengine.SearchTaskResult;

import java.io.Serializable;
import java.util.*;

public class DivideAndScheduleMovables {
    private final ArrayList<Agent> agents_scheduled;
    private HungarianAlgorithmResizable hungarian_algorithm;

    private static Set<Integer> boxes_ids;
    private static Set<Integer> boxes_ids_scheduled;

    //the goals that agents had solved are added here
    private static HashMap<Integer, ArrayList<int[]> > agents_to_solved_goals;

    //jobs are columns
    //costs are rows
    //agents and boxes should have the same color
    //we choose columns for agents
    //we choose rows for total path from agent to box to boxs goal
    public DivideAndScheduleMovables(Set<Integer> boxesIds) {
        boxes_ids = boxesIds;

        boxes_ids_scheduled = new HashSet<>();
        this.agents_scheduled = new ArrayList<>();
        agents_to_solved_goals = new HashMap<>();

    }


    public ArrayDeque<Agent> getAgentsScheduledRandom(LinkedList<Agent> agents){
        ArrayDeque<Agent> agents_to_schedule = new ArrayDeque<>();

        Box next_box;
        for (Integer next_box_id: boxes_ids){
            next_box = MapFixedObjects.getBoxByID(next_box_id);
            int next_box_color = next_box.getColor();

            for (Agent agent : agents) {
                if(agent.getColor() == next_box_color){
                    boxes_ids.remove(next_box_id);
                    boxes_ids_scheduled.add(next_box_id);
                    agent.setGoalStopPosition( next_box.getGoalPosition() );
                    agents_to_schedule.add(agent);
                    this.agents_scheduled.add(agent);
                }
            }
        }
        return agents_to_schedule;
    }

    //input the agents to schedule and and has the result a pair of agent and box , and all
    //the pairs are included in MovablesScheduling
    public MovablesScheduling getAgentsScheduled(LinkedList<Agent> agents_to_schedule){
        final int AGENTS = 0;
        final int BOXES = 1;

        HashMap<Integer, ArrayList<Agent>> groups_agents = new HashMap<>();
        HashMap<Integer, Serializable[][]> groups_movables = new HashMap<>(groups_agents.size());

        for (Agent agent : agents_to_schedule) {
            if (groups_agents.containsKey(agent.getColor())) {
                groups_agents.get(agent.getColor()).add(agent);
             }else {
                ArrayList<Agent> arrayList = new ArrayList<>();
                arrayList.add(agent);
                groups_agents.put(agent.getColor(),arrayList);
            }
        }

        for (Integer key_color : groups_agents.keySet()){
            ArrayList<Agent> _agents = groups_agents.get(key_color);
            Integer[][] match_movables = new Integer[2][];
            match_movables[AGENTS] = new Integer[_agents.size()];

            for (int i = 0; i < _agents.size(); i++) {
                int mark_id = _agents.get(i).getNumberMark();
                match_movables[AGENTS][i] = mark_id;
            }

            ArrayDeque<Integer> boxes_by_color = MapFixedObjects.getBoxesIDsByColor(key_color, boxes_ids);
            match_movables[BOXES] = new Integer[boxes_by_color.size()];

            int index_box = 0;
            while (!boxes_by_color.isEmpty())
                match_movables[BOXES][index_box++] = boxes_by_color.pop();

            groups_movables.put(key_color, match_movables);
        }

         MovablesScheduling movablesScheduling = new MovablesScheduling();

        //build matrix with heuristic costs
        for (Integer color : groups_movables.keySet()){
            Integer[][] _group = (Integer[][]) groups_movables.get(color);

            int rows = _group[BOXES].length;
            int columns = _group[AGENTS].length;
            int[][] heuristc_costs = new int[rows][columns];
            Integer[] agentss =_group[AGENTS];
            Integer[] boxxxes = _group[BOXES];

            Box box_to_next;
            Agent agent;
            for (int box_no = 0; box_no < boxxxes.length ; box_no++) {
                box_to_next = MapFixedObjects.getBoxByID(boxxxes[box_no]);

                for (int agent_i = 0; agent_i < agentss.length ; agent_i++) {
                    agent = MapFixedObjects.getByAgentMarkId(agentss[agent_i]);

                    int heuristic_value = box_to_next.getCostHeuristic() + MapFixedObjects.getManhattenHeuristic(agent.getCoordinates(), box_to_next.getCoordinates());
                    heuristc_costs[box_no][agent_i] = heuristic_value;
                }
            }

            if(_group[AGENTS].length > 1) {
                this.hungarian_algorithm = new HungarianAlgorithmResizable(heuristc_costs);
                ArrayList<int[]> values_resulted = this.hungarian_algorithm.getMinimumCost();

                for(int[] pair : values_resulted){
                    int agent_index = pair[0];
                    int box_index = pair[1];
                    Integer agent_opt = agentss[agent_index];
                    Integer box_opt = boxxxes[box_index];

                    movablesScheduling.setUpPair(agent_opt, box_opt);

                    boxes_ids.remove(box_opt);
                    boxes_ids_scheduled.add(box_opt);

                    Agent __agent = MapFixedObjects.getByAgentMarkId(agent_opt);
                    agents_scheduled.add(__agent);
                    agents_to_schedule.remove(__agent);
                }

            }else{
                int min_heuristic_value = heuristc_costs[0][0];
                int box_no = 0;
                for (box_no = 0; box_no < heuristc_costs.length ; box_no++) {
                    if (min_heuristic_value > heuristc_costs[box_no][0]){
                        min_heuristic_value = heuristc_costs[box_no][0];
                    }
                }

                Integer agent_opt = agentss[0];
                Integer box_opt = boxxxes[0];

                movablesScheduling.setUpPair(agent_opt, box_opt);
                boxes_ids.remove(box_opt);
                boxes_ids_scheduled.add(box_opt);

                Agent __agent = MapFixedObjects.getByAgentMarkId(agent_opt);
                agents_scheduled.add(__agent);
                agents_to_schedule.remove(__agent);
             }
         }

        return movablesScheduling;
    }

    public ArrayDeque<Box> getBoxesScheduled(){
        ArrayDeque<Box> boxes_to_schedule = new ArrayDeque<>();
        Box box;
        for (Agent agent : this.agents_scheduled) {
            if(agent.getSolvedStatus() == SolvedStatus.GOAL_STEP_SOLVED ){
                for(Integer next_box : boxes_ids){
                    box = MapFixedObjects.getBoxByID(next_box);
                    if (Arrays.equals(agent.getGoalStopPosition(), box.getCoordinates())){
                        boxes_to_schedule.add(box);
                    }
                }
            }
        }
        return boxes_to_schedule;
    }

    //is only used only after calling the method getAgentsScheduled
    public TaskScheduled getSearchResults(){
        HashMap<Integer,ArrayDeque<Integer>> agents_to_boxes = new HashMap<>();

        ArrayList<Integer> agents_solved_mark_ids = new ArrayList<>();
        ArrayList<Integer> agts_total = new ArrayList<>();
        ArrayList<Integer> bxs_total = new ArrayList<>();

        for (Agent agent : this.agents_scheduled) {
            switch (agent.getSolvedStatus()) {
                case GOAL_STEP_SOLVED:
                    ArrayDeque<Integer> boxes_solved = new ArrayDeque<>() ;
                    for(Integer next_box : boxes_ids_scheduled){
                        Box box = MapFixedObjects.getBoxByID(next_box);
                        ArrayList<int[]> goals_sloved = agents_to_solved_goals.get(agent.getNumberMark());
                        int[] last_goal = goals_sloved.get(goals_sloved.size() - 1); //check last goal solved if matches the box
                        if (Coordinates.getRow(last_goal) == Coordinates.getRow(box.getNeighbourGoal())
                            && Coordinates.getCol(last_goal) == Coordinates.getCol(box.getNeighbourGoal())
                        ){
                            boolean is_changed = agent.updatePositionCoordinates();

                            boxes_solved.add(box.getLetterMark());
                            agts_total.add(agent.getNumberMark());
                            bxs_total.add(next_box);
                        }
                    }
                    if(boxes_solved.size() > 0)
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
        taskScheduled.addValidMovables(agts_total, bxs_total);

        taskScheduled.addAggentsBoxes(agents_to_boxes);
        taskScheduled.addAgents(agents_solved_mark_ids);

        return taskScheduled;
    }

    private void setAgentsGoalsFound(int[] start_group_agents, ArrayDeque<int[]> path_found) {
        if(path_found.size()==0) return;

        int[] goal_found = path_found.peek();

        for (int i = 0; i < start_group_agents.length; i++) {
            int agt_id = start_group_agents[i];

            if (agents_to_solved_goals.containsKey(agt_id)){
                agents_to_solved_goals.get(agt_id).add(Coordinates.getCoordinatesAt(i, goal_found));
            }else {
                ArrayList<int[]> goals_solved = new ArrayList<>();
                goals_solved.add(Coordinates.getCoordinatesAt(i, goal_found));
                agents_to_solved_goals.put(agt_id, goals_solved);
            }
        }
    }

    public void setAgentsGoalsFound(SearchTaskResult searchTaskResult) {
        this.setAgentsGoalsFound(searchTaskResult.getStartGroupAgents(), searchTaskResult.getPath());


    }
}




