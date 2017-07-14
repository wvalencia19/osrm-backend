#ifndef GEOMETRY_COMPRESSOR_HPP
#define GEOMETRY_COMPRESSOR_HPP

#include "util/typedefs.hpp"

#include "util/node_based_graph.hpp"

#include <memory>
#include <unordered_set>
#include <vector>

namespace osrm
{
namespace extractor
{

class CompressedEdgeContainer;
struct TurnRestriction;

class GraphCompressor
{
    using EdgeData = util::NodeBasedDynamicGraph::EdgeData;

  public:
    void Compress(const std::unordered_set<NodeID> &barrier_nodes,
                  const std::unordered_set<NodeID> &traffic_lights,
                  std::vector<TurnRestriction> &turn_restrictions,
                  util::NodeBasedDynamicGraph &graph,
                  CompressedEdgeContainer &geometry_compressor);

  private:
    void PrintStatistics(unsigned original_number_of_nodes,
                         unsigned original_number_of_edges,
                         const util::NodeBasedDynamicGraph &graph) const;
};
}
}

#endif
