package org.agents.planning;

import org.agents.Agent;
import org.agents.Box;
import org.agents.MapFixedObjects;
import org.agents.planning.schedulling.TaskScheduled;

import java.util.*;

public class MovablesScheduling {
   private final ArrayList<Agent> agents_scheduled;
    private final LinkedList<Box> boxes_scheduled;

    private HashMap<Integer,Integer> boxes_heuristic;

    public MovablesScheduling() {
        this.agents_scheduled = new ArrayList<>();
        this.boxes_scheduled = new LinkedList<>();
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
                    for(Box next_box : boxes_scheduled){
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

    //agent has target box
    public void setUpPair(Integer agent_id, Integer box_target_id) {
        Agent agent = MapFixedObjects.getByAgentMarkId(agent_id);
        Box box_target = MapFixedObjects.getBoxByID(box_target_id);

        agent.setGoalPosition(box_target.getCoordinates());
        this.agents_scheduled.add(agent);
        this.boxes_scheduled.add(box_target);
    }

    //update agents goal status and then querry the same class for getSearchResults()
    public ArrayList<Agent> getAgentsScheduled(){
        return this.agents_scheduled;
    }

    public LinkedList<Box> getBoxesScheduled(){
        return this.boxes_scheduled;
    }

}
