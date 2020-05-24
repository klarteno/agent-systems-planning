package org.agents.planning.schedulling;

import java.util.*;

public class TrackedGroups {
    private final Set<Integer> agents_scheduled_ids;
    private final Set<Integer> boxes_scheduled_ids;
    //this table stores for every number mark of the movable objectss a index that is used to index in the table_for_paths
    private final Collection<Integer> movables_ids;
    private  HashMap<Integer,Integer> ids_indexes;//should be declared final
    private final Set<Integer> ungrouped_movables;

    public TrackedGroups(Set<Integer> agentsScheduledIds, Set<Integer> boxesScheduledIds) {
        this.agents_scheduled_ids = agentsScheduledIds;
        this.boxes_scheduled_ids = boxesScheduledIds;

        this.movables_ids = new ArrayList<>();
        this.movables_ids.addAll(this.agents_scheduled_ids);
        this.movables_ids.addAll(this.boxes_scheduled_ids);

        this.ungrouped_movables = new HashSet<>(this.movables_ids.size());
        ungrouped_movables.addAll(this.movables_ids);
        
        initIdsIndexes();//indexes the mark_ids and stores a index
    }

    private void initIdsIndexes(){
        this.ids_indexes = new HashMap<>(this.movables_ids.size());
        int index = 0;
        for (Integer next : movables_ids){
            ids_indexes.put(next, index++);
        }
    }

    public Set<Integer> getAgentsScheduledIds() {
        return this.agents_scheduled_ids;
    }

    public Set<Integer> getBoxesScheduled_Ids() {
        return this.boxes_scheduled_ids;
    }

    public Collection<? extends Integer> getTrackedIds() {
        return  this.movables_ids;
    }

    public int getIndexFor(int mark_id) {
        return this.ids_indexes.get(mark_id);
    }

    public int getGroupSize(){
            return this.movables_ids.size();
    }
    
    public Set<Integer> getAllUnGroupedIDs(){
        return this.ungrouped_movables;
    }
}
