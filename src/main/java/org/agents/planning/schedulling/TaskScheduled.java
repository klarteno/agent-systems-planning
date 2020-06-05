package org.agents.planning.schedulling;

import org.agents.Agent;
import org.agents.Box;

import java.util.*;

public class TaskScheduled {
    private int[] group_marks_total;
    private ArrayDeque<int[]> group_marks_total_path;

    private HashMap<Integer, ArrayDeque<Integer> > agents_to_boxes;

    private final HashMap<Integer, ArrayDeque<int[]> > agents_to_paths;
    private final HashMap<Integer, ArrayDeque<int[]> > boxes_to_paths;

    private ArrayList<Integer> agents_solved_mark_ids;

    public TaskScheduled() {
        agents_to_paths = new HashMap<>();
        agents_to_boxes = new HashMap<>();
        boxes_to_paths = new HashMap<>();
    }

    //from here the other hashmaps can be constructed
    //represent the the main entry call to this class until refactoring
    public TaskScheduled(int[] group_marks, ArrayDeque<int[]> new_path) {
        this.group_marks_total = group_marks;
        this.group_marks_total_path = new_path;

        agents_to_paths = new HashMap<>();
        agents_to_boxes = new HashMap<>();
        boxes_to_paths = new HashMap<>();
    }

    //if this used is very likely that the hash_maps from this class have inconsistent information
    //so use the other with care
    public int[] getGroupsTotalMarks(){
        return this.group_marks_total;
    }

    //if this used is very likely that the hash_maps from this class have inconsistent information
    //so use the other with care
    public ArrayDeque<int[]> getGroupsTotalPath(){
        return this.group_marks_total_path;
    }

    public Set<Integer> getGroupMarksSolved(){
        Set<Integer>  group_marks = new HashSet<>();
        group_marks.addAll(agents_to_paths.keySet());
        group_marks.addAll(boxes_to_paths.keySet());

        return group_marks;
       //return group_marks.toArray(new Integer[0]);
    }

    public boolean isTheSameGroupAs(int[] group_test){
        if(this.group_marks_total.length == 0){
            Set<Integer>  group_marks = new HashSet<>();
            for (int value : group_test) {
                group_marks.add(value);
            }
            Set<Integer> __groups = this.getGroupMarksSolved();

            if (group_marks.size() != __groups.size()) return false;
            boolean __difference_set = group_marks.removeAll(__groups);

            int index = 0;
            this.group_marks_total = new int[__groups.size()];
            for (Integer id_value : __groups) {
                this.group_marks_total[index++] = id_value;
             }

            return group_marks.size() == 0;
        }else
            return Arrays.equals(this.group_marks_total, group_test);
    }


    public void addAgents(ArrayList<Integer> agents_solved_mark_ids) {
        this.agents_solved_mark_ids = agents_solved_mark_ids;
    }

    public boolean isSchedulable(){
        boolean result = false;
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

    public void replacePathsFor(int[] group_marks, ArrayDeque<int[]> new_path) {
        this.group_marks_total = group_marks;
        this.group_marks_total_path = new_path;
    }
}
