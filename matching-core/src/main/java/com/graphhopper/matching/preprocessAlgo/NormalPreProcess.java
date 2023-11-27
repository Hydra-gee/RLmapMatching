package com.graphhopper.matching.preprocessAlgo;

import com.graphhopper.matching.RLMM;
import com.graphhopper.matching.entities.Observation;
import com.graphhopper.matching.entities.ObservationWithCandidateStates;
import com.graphhopper.routing.querygraph.QueryGraph;
import com.graphhopper.routing.util.DefaultEdgeFilter;
import com.graphhopper.storage.index.Snap;

import java.util.Collection;
import java.util.List;
import java.util.Map;
import java.util.stream.Collectors;

public class NormalPreProcess extends BasePreProcess {
    public NormalPreProcess(RLMM rlmm) {
        super(rlmm);
    }

    public Collection<Snap> deduplicate(Collection<Snap> splits) {
        // Only keep one split per node number. Let's say the last one.
        Map<Integer, Snap> splitsByNodeNumber = splits.stream().collect(Collectors.toMap(Snap::getClosestNode, s -> s, (s1, s2) -> s2));
        return splitsByNodeNumber.values();
    }

    @Override
    public List<ObservationWithCandidateStates> preprocess(List<Observation> observations) {
        List<Observation> filteredObservations = filterObservations(observations);
        List<Collection<Snap>> splitsPerObservation = filteredObservations.stream().map(o -> rlmm.locationIndex.findNClosest(o.getPoint().lat, o.getPoint().lon, DefaultEdgeFilter.allEdges(rlmm.weighting.getFlagEncoder()), rlmm.measurementErrorSigma))
                .collect(Collectors.toList());
        rlmm.queryGraph = QueryGraph.create(rlmm.graph, splitsPerObservation.stream().flatMap(Collection::stream).collect(Collectors.toList()));
        splitsPerObservation = splitsPerObservation.stream().map(this::deduplicate).collect(Collectors.toList());
        return createTimeSteps(filteredObservations, splitsPerObservation);
    }
}
