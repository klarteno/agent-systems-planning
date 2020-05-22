package org.agents.planning.schedulling;

import org.agents.Agent;
import org.agents.Box;

import java.util.*;

public class TaskScheduled {
    private HashMap<Integer, ArrayDeque<Integer> > agents_to_boxes;

    private final HashMap<Integer, ArrayDeque<int[]> > agents_to_paths;
    private final HashMap<Integer, ArrayDeque<int[]> > boxes_to_paths;

    private ArrayList<Integer> agents_solved_mark_ids;

    public TaskScheduled() {
        agents_to_paths = new HashMap<>();
        agents_to_boxes = new HashMap<>();
        boxes_to_paths = new HashMap<>();
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

            return this.agents_to_boxes.entrySet();
    }

    public ArrayList<Integer> getAgentsSolved(){
        if (this.agents_solved_mark_ids != null){
            return this.agents_solved_mark_ids;
        }
        return null;
    }

    public void add(Agent agent, ArrayDeque<int[]> agent_path) {
        this.agents_to_paths.put(agent.getNumberMark(), agent_path);
    }

    public void add(Box box, ArrayDeque<int[]> box_path) {
        this.boxes_to_paths.put(box.getLetterMark(), box_path);
    }

    public HashMap<Integer, ArrayDeque<int[]>> getAgentsToPaths() {
        return this.agents_to_paths;
    }

    public HashMap<Integer, ArrayDeque<int[]>> getBoxesToPaths() {
        return this.boxes_to_paths;
    }
}
