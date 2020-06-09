package org.agents.planning.schedulling;

import org.agents.Agent;
import org.agents.Box;
import org.agents.MapFixedObjects;
import org.agents.markings.Coordinates;
import org.agents.markings.SolvedStatus;
import org.agents.searchengine.SearchTaskResult;

import java.util.*;

public final class MovablesScheduling {
    private final ArrayList<Agent> agents_scheduled;
    private final LinkedList<Box> boxes_scheduled;

    private final Set<Integer> agents_scheduled_ids;
    private final Set<Integer> boxes_scheduled_ids;

    private final HashMap<Integer, Set<Integer> > agents_ids_to_boxes_ids;

    public MovablesScheduling() {
        this.agents_scheduled = new ArrayList<>();
        this.boxes_scheduled = new LinkedList<>();
        this.agents_scheduled_ids = new HashSet<>();
        this.boxes_scheduled_ids = new HashSet<>();

        agents_ids_to_boxes_ids = new HashMap<>();
     }

    public boolean setAgentsScheduledSolvedResults(SearchTaskResult searchTaskResult, int[] final_agents_position, SolvedStatus solvedStatus) {
        int[] group_agents = searchTaskResult.getStartGroupAgents();

        boolean status_changed = false;
        for (Agent agent: this.agents_scheduled){
            for (int group_index = 0; group_index < group_agents.length; group_index++) {
                if (agent.getNumberMark() == group_agents[group_index]) {
                    SolvedStatus solved_status = agent.getSolvedStatus();
                    if (agent.getSolvedStatus() != solvedStatus) {
                        agent.setGoalPosition(Coordinates.getCoordinatesAt(group_index, final_agents_position));
                        agent.setSolvedStatus(solvedStatus);
                        status_changed = true;
                    }
                }
            }
        }
        return status_changed;
    }

    public TaskScheduled getSearchResults(){
        
        HashMap<Integer,ArrayDeque<Integer>> agents_to_boxes = new HashMap<>();
        ArrayList<Integer> agents_solved_mark_ids = new ArrayList<>();

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
        box_target.setSolvedStatus(SolvedStatus.IN_USE);

        agent.setGoalPosition(box_target.getCoordinates());
        agent.setSolvedStatus(SolvedStatus.IN_USE);

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

    public Set<Integer> getAgentsIds(){
        return new HashSet<>(this.agents_scheduled_ids);
    }

    public Set<Integer> getBoxesIds(){
        return new HashSet<>(this.boxes_scheduled_ids);
    }

    //call this method only after calling the method getStartGroupAgentsBoxes_ToSearch()
    public HashMap<Integer, Set<Integer>> getAgentsIdsToBoxesIds(){
        return this.agents_ids_to_boxes_ids;
    }
    
    //after this call getAgentsIdxsToBoxesIdxs()
    public SearchScheduled getStartGroupAgentsBoxes_ToSearch(){
        //final int INDEX_OF_BOXES = 2;
        final int INDEX_OF_AGENTS = 0;
        final int START_GROUP_AGENTS = 1;
        final int INDEX_OF_GROUP = 2;

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

        return getMatchedAgentsBoxesIndexes2( agents_group_ids, box_group_ids, this.agents_ids_to_boxes_ids);
    }

    public SearchScheduled getMatchedAgentsBoxesIndexes(TaskScheduled task_scheduled){
        int[] agents_group_ids = task_scheduled.getValidAgents();
        int[] box_group_ids =  task_scheduled.getValidBoxes();


        for (int ag_i = 0; ag_i < agents_group_ids.length; ag_i++) {
            int ag_id = agents_group_ids[ag_i];
            Set<Integer> boxes_ids = this.agents_ids_to_boxes_ids.get(ag_id);
            for (int i = 0; i < box_group_ids.length; i++) {
                int bx_id = box_group_ids[i];
                if (boxes_ids.contains(bx_id)) {
                    Agent agt = (Agent) MapFixedObjects.getByMarkNo(ag_id);
                    Box bx = (Box) MapFixedObjects.getByMarkNo(bx_id);
                    agt.setGoalPosition(bx.getCoordinates());
                }
            }
        }

        return getMatchedAgentsBoxesIndexes2(agents_group_ids, box_group_ids, this.agents_ids_to_boxes_ids);
    }

    private static SearchScheduled getMatchedAgentsBoxesIndexes2(int[] agents_group_ids, int[] box_group_ids, HashMap<Integer, Set<Integer>> agents_ids_to_boxes_ids){

        SearchScheduled searchScheduled = new SearchScheduled();

        int[][]  total_group = new int[3][];
        searchScheduled.setTotalGroup(total_group);

        HashMap<Integer, int[]> agents_idxs_to_boxes_idxs = new HashMap<>();
        searchScheduled.setAgentsIdxsToBoxesIdxs(agents_idxs_to_boxes_idxs);

        Arrays.parallelSort(agents_group_ids);
        Arrays.parallelSort(box_group_ids);

        int[] index_agents = new int[agents_group_ids.length];
        total_group[SearchScheduled.INDEX_OF_AGENTS] = index_agents;
        int[] start_group = new int[agents_group_ids.length + box_group_ids.length];
        int[] start_group_agents = new int[agents_group_ids.length];
        total_group[SearchScheduled.INDEX_OF_GROUP] = start_group;
        total_group[SearchScheduled.START_GROUP_AGENTS]= start_group_agents;

        int index = 0;
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

        for (int bx_id : box_group_ids) {
            start_group[index++] = bx_id;
        }

        return  searchScheduled;
    }

}
