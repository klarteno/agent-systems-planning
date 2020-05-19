package org.agents.planning.schedulling;

import java.util.*;

public class TaskScheduled {
    private HashMap<Integer, ArrayDeque<Integer>> agents_to_boxes = null;
    private ArrayList<Integer> agents_solved_mark_ids;

    public TaskScheduled() {
    }

    public void addAgents(ArrayList<Integer> agents_solved_mark_ids) {
        this.agents_solved_mark_ids = agents_solved_mark_ids;
    }

    public boolean isSchedulable(){
        boolean result = false;
        if (this.agents_to_boxes != null)
           result = result || agents_to_boxes.size() > 0;

        if (this.agents_solved_mark_ids != null)
            result = result || agents_solved_mark_ids.size() > 0;

        return result;
    }

    public void addAggentsBoxes(HashMap<Integer, ArrayDeque<Integer>> agents_to_boxes) {
        this.agents_to_boxes = agents_to_boxes;
    }


    public Set<Map.Entry<Integer, ArrayDeque<Integer>>> getAgentsToBoxes() {
        //Integer key_value = this.agents_to_boxes.keySet().iterator().next();
        //ArrayDeque<Integer> boxes = this.agents_to_boxes.remove(key_value);
        if (this.agents_to_boxes != null) {
            return this.agents_to_boxes.entrySet();
        }
        return null;
    }

    public ArrayList<Integer> getAgentsSolved(){
        if (this.agents_solved_mark_ids != null){
            return this.agents_solved_mark_ids;
        }

        return null;
    }
}
