package org.agents.planning;

import org.agents.Agent;
import org.agents.Box;
import org.agents.MapFixedObjects;
import org.agents.markings.SolvedStatus;
import org.agents.planning.conflicts.ConflictAvoidanceCheckingRules;
import org.agents.planning.schedulling.TaskScheduled;

import java.util.*;

public class MovablesScheduling {
   private final ArrayList<Agent> agents_scheduled_opt;
    private final  LinkedList<Box> boxes;

    private HashMap<Integer,Integer> boxes_heuristic;

    public MovablesScheduling() {
        this.boxes = new LinkedList<>();
        this.agents_scheduled_opt = new ArrayList<>();//at least the number of boxes
    }

    public TaskScheduled getSearchResults(){
        HashMap<Integer,ArrayDeque<Integer>> agents_to_boxes = new HashMap<>();

        ArrayList<Integer> agents_solved_mark_ids = new ArrayList<>();
        Set<Map.Entry<Integer, ArrayDeque<Integer>>> round = agents_to_boxes.entrySet();
        ArrayList<Integer> boxes_solved_mark_ids = new ArrayList<>();

        for (Agent agent : this.agents_scheduled_opt) {
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

    //agent has target box
    public void setUpPair(Agent agent, Box box_target) {
        agent.setGoalPosition(box_target.getCoordinates());
        this.agents_scheduled_opt.add(agent);
        this.boxes.add(box_target);
    }

    //update agents goal status and then querry the same class for getSearchResults()
    public ArrayList<Agent> getAgentsScheduled(){
        return this.agents_scheduled_opt;
    }

}
