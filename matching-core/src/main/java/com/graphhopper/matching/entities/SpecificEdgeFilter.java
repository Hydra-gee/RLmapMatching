package com.graphhopper.matching.entities;

import com.carrotsearch.hppc.IntArrayList;
import com.graphhopper.routing.util.EdgeFilter;
import com.graphhopper.util.EdgeIteratorState;

import java.util.Collection;
import java.util.List;
import java.util.stream.Collectors;

public class SpecificEdgeFilter implements EdgeFilter {
    private List<EdgeIteratorState> acceptEdges;
    private List<Integer> acceptEdgeIds;
    public SpecificEdgeFilter(List<EdgeIteratorState> acceptEdges){
        this.acceptEdges = acceptEdges;
        this.acceptEdgeIds = this.acceptEdges.stream().map(s->s.getEdge()).collect(Collectors.toList());
    }

    @Override
    public boolean accept(EdgeIteratorState edgeIteratorState) {
//        return acceptEdges.contains(edgeIteratorState);
        return acceptEdgeIds.contains(edgeIteratorState.getEdge());
    }
}
