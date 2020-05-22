package org.agents.planning.schedulling;

import java.util.Collection;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Set;

public class TrackedGroups {
    //this table stores for every number mark of the movable objectss a index that is used to index in the table_for_paths
    private Collection<? extends Integer> movables_ids;
    private HashMap<Integer,Integer> ids_indexes;//should be declared final
    private final Set<Integer> ungrouped_movables;

    public TrackedGroups(Collection<? extends Integer> movablesIds) {
        this.movables_ids = movablesIds;
        this.ungrouped_movables = new HashSet<>(this.movables_ids.size());
        ungrouped_movables.addAll(movablesIds);
        
        initIdsIndexes();//indexes the mark_ids and stores a index
    }

    public void setUpTracked(Collection<? extends Integer> movables_ids){
        //table_for_paths = new int[number_of_movables][][];
        this.movables_ids = movables_ids;
        initIdsIndexes();
    }

    private void initIdsIndexes(){
        this.ids_indexes = new HashMap<>(this.movables_ids.size());
        int index = 0;
        for (Integer next : movables_ids){
            ids_indexes.put(next, index++);
        }
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
