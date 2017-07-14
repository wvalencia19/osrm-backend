#ifndef OSRM_EXTRACTOR_WAY_RESTRICTION_MAP_HPP_
#define OSRM_EXTRACTOR_WAY_RESTRICTION_MAP_HPP_

#include <utility>
#include <vector>

// to access the turn restrictions
#include <boost/unordered_map.hpp>

#include "extractor/restriction.hpp"
#include "util/typedefs.hpp"

// Given the compressed representation of via-way turn restrictions, we provide a fast access into
// the restrictions to indicate which turns may be restricted due to a way in between
namespace osrm
{
namespace extractor
{

class WayRestrictionMap
{
  public:
    struct ViaWay
    {
        std::size_t id;
        NodeID from;
        NodeID to;
    };
    WayRestrictionMap(const std::vector<TurnRestriction> &turn_restrictions);

    bool IsViaWay(const NodeID from, const NodeID to) const;

    // check if an edge between two nodes is a restricted turn
    bool IsStart(const NodeID from, const NodeID to) const;
    bool IsEnd(const NodeID from, const NodeID to) const;

    std::size_t Size() const;

    // number of duplicated nodes
    std::size_t NumberOfDuplicatedNodes() const;

    // find the ID of the duplicated node (zero based) for a given restriction id
    std::size_t DuplicatedNodeID(const std::size_t restriction_id) const;

    // returns a representative for the duplicated way, consisting of the representative ID (first
    // ID of the nodes restrictions) and the from/to vertices of the via-way
    std::vector<ViaWay> DuplicatedNodeRepresentatives() const;

    std::vector<std::size_t> GetIDs(const NodeID from, const NodeID to) const;

    TurnRestriction const &GetRestriction(std::size_t) const;

  private:
    // access all restrictions that have the same starting way and via way. Any duplicated node
    // represents the same in-way + via-way combination. This vector contains data about all
    // restrictions and their assigned duplicated nodes. It indicates the minimum restriciton ID
    // that is represented by the next node. The ID of a node is defined as the position of the
    // lower bound of the restrictions ID within this array
    std::vector<std::size_t> duplicated_node_groups;

    boost::unordered_multimap<std::pair<NodeID, NodeID>, std::size_t> restriction_starts;
    boost::unordered_multimap<std::pair<NodeID, NodeID>, std::size_t> restriction_ends;
    boost::unordered_multimap<std::pair<NodeID, NodeID>, std::size_t> via_ways;

    std::vector<TurnRestriction> restriction_data;
};

} // namespace extractor
} // namespace osrm

#endif // OSRM_EXTRACTOR_WAY_RESTRICTION_MAP_HPP_
