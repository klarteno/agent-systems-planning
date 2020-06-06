package org.agents.planning;

import org.agents.Agent;
import org.agents.Box;
import org.agents.MapFixedObjects;
import org.agents.planning.schedulling.TaskScheduled;
import org.agents.planning.schedulling.TrackedGroups;

import java.util.*;

public final class MovablesScheduling {
    private final ArrayList<Agent> agents_scheduled;
    private final LinkedList<Box> boxes_scheduled;

    private HashMap<Integer,Integer> boxes_heuristic;
    private final Set<Integer> agents_scheduled_ids;
    private final Set<Integer> boxes_scheduled_ids;

    private final HashMap<Integer, Set<Integer> > agents_ids_to_boxes_ids;

    private boolean total_group_unchanged;
    private int[][] total_group;
    private HashMap<Integer, int[]> agents_idxs_to_boxes_idxs;

    public MovablesScheduling() {
        this.agents_scheduled = new ArrayList<>();
        this.boxes_scheduled = new LinkedList<>();
        this.agents_scheduled_ids = new HashSet<>();
        this.boxes_scheduled_ids = new HashSet<>();

        agents_ids_to_boxes_ids = new HashMap<>();
        total_group_unchanged = false;
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

    public TrackedGroups getTrackedGroups(){
        return new TrackedGroups(this.agents_scheduled_ids, this.boxes_scheduled_ids );
    }

    //agent has target box
    public void setUpPair(Integer agent_id, Integer box_target_id) {

        if(agents_ids_to_boxes_ids.containsKey(agent_id)){
            agents_ids_to_boxes_ids.get(agent_id).add(box_target_id);
        }else {
            Set<Integer> box_ids = new HashSet<>();
            box_ids.add(box_target_id);
            agents_ids_to_boxes_ids.put(agent_id, box_ids);
        }

        this.agents_scheduled_ids.add(agent_id);
        this.boxes_scheduled_ids.add(box_target_id);

        Agent agent = MapFixedObjects.getByAgentMarkId(agent_id);
        Box box_target = MapFixedObjects.getBoxByID(box_target_id);

        agent.setGoalPosition(box_target.getCoordinates());
        this.agents_scheduled.add(agent);
        this.boxes_scheduled.add(box_target);

        total_group_unchanged = false;
    }

    //update agents goal status and then querry the same class for getSearchResults()
    public ArrayList<Agent> getAgentsScheduled(){
        return this.agents_scheduled;
    }

    public LinkedList<Box> getBoxesScheduled(){
        return this.boxes_scheduled;
    }

    public Set<Integer> getAgentsIds(){
        return new HashSet<>(this.agents_scheduled_ids);
    }

    public Set<Integer> getBoxesIds(){
        return new HashSet<>(this.boxes_scheduled_ids);
    }

    //call this method only after calling the method getStartGroupAgentsBoxes_ToSearch()
    public HashMap<Integer, int[]> getAgentsIdxsToBoxesIdxs(){
        return this.agents_idxs_to_boxes_idxs;
    }

    public int[][] getStartGroupAgentsBoxes_ToSearch(){
        //final int INDEX_OF_BOXES = 2;
        final int INDEX_OF_AGENTS = 0;
        final int START_GROUP_AGENTS = 1;
        final int INDEX_OF_GROUP = 2;

        if(!total_group_unchanged){
            total_group = new int[3][];

            agents_idxs_to_boxes_idxs = new HashMap<>();

            int[] box_group_ids = new  int[this.boxes_scheduled_ids.size()];
            int index = 0;
            for (Integer bx_id : this.boxes_scheduled_ids ){
                box_group_ids[index++] = bx_id;
            }
            index = 0;
            int[] agents_group_ids = new  int[this.agents_scheduled_ids.size()];
            for (Integer ag_id : this.agents_scheduled_ids ){
                agents_group_ids[index] = ag_id;
            }

            Arrays.parallelSort(agents_group_ids);
            Arrays.parallelSort(box_group_ids);

            int[] index_agents = new int[agents_group_ids.length];
            total_group[INDEX_OF_AGENTS] = index_agents;
            int[] start_group = new int[agents_group_ids.length + box_group_ids.length];
            int[] start_group_agents = new int[agents_group_ids.length];
            total_group[INDEX_OF_GROUP]= start_group;
            total_group[START_GROUP_AGENTS]= start_group_agents;


            index = 0;
            for (int i = 0; i < agents_group_ids.length; i++) {
                index_agents[i] = index;

                int ag_id = agents_group_ids[i];
                Set<Integer> box_set = agents_ids_to_boxes_ids.get(ag_id);

                int[] box_indxs = new int[box_set.size()];
                int box_indxs_index = 0;
                for (int b_i = 0; b_i < box_group_ids.length; b_i++) {
                    int b_id = box_group_ids[b_i];
                    for (Integer b_id2 : box_set){
                        if (b_id == b_id2){
                            box_indxs[box_indxs_index++] = index_agents.length + b_i;
                        }
                    }
                }

                agents_idxs_to_boxes_idxs.put(index, box_indxs) ;
                start_group_agents[index] = ag_id;
                start_group[index++] = ag_id;
            }

            for (int i = 0; i < box_group_ids.length; i++) {
                int bx_id = box_group_ids[i];

                start_group[index++] = bx_id;
            }

            total_group_unchanged = true;
        }

        return  total_group;
    }



    private int[][] getStartGroupAgentsBoxes_ToSearchBackup(){
        //final int INDEX_OF_BOXES = 2;
        final int INDEX_OF_AGENTS = 0;
        final int INDEX_OF_GROUP = 1;

        if(total_group_unchanged){
            total_group = new int[2][];

            HashMap<Integer, int[]> agents_idxs_to_boxes_idxs = new HashMap<>();


            int[] index_agents = new int[this.agents_scheduled_ids.size()];
            total_group[INDEX_OF_AGENTS] = index_agents;

            int[] start_group = new int[this.agents_scheduled_ids.size() + this.boxes_scheduled_ids.size()];
            total_group[INDEX_OF_GROUP]= start_group;

            int index = 0;
            for (Integer ag_id : this.agents_scheduled_ids ){
                index_agents[index] = ag_id;

                int[] box_ids = new int[agents_ids_to_boxes_ids.get(ag_id).size()];
                agents_idxs_to_boxes_idxs.put(index, box_ids) ;

                start_group[index++] = ag_id;
            }

            for (Integer bx_id : this.boxes_scheduled_ids ){
                start_group[index++] = bx_id;
            }

            total_group_unchanged = true;
        }

        return  total_group;
    }

}
