#include "extractor/way_restriction_map.hpp"

#include <iterator>
#include <utility>

namespace osrm
{
namespace extractor
{

WayRestrictionMap::WayRestrictionMap(const std::vector<TurnRestriction> &turn_restrictions)
{
    // get all way restrictions
    const auto get_way_restrictions = [this](const auto &turn_restriction) {
        if (turn_restriction.Type() == RestrictionType::WAY_RESTRICTION)
            restriction_data.push_back(turn_restriction);
    };
    std::for_each(turn_restrictions.begin(), turn_restrictions.end(), get_way_restrictions);

    const auto as_duplicated_node = [](auto const &restriction) {
        auto &way = restriction.AsWayRestriction();
        return std::make_pair(std::make_pair(way.in_restriction.from, way.in_restriction.via),
                              way.out_restriction.via);
    };

    const auto by_duplicated_node = [&](auto const &lhs, auto const &rhs) {
        return as_duplicated_node(lhs) < as_duplicated_node(rhs);
    };

    std::sort(restriction_data.begin(), restriction_data.end(), by_duplicated_node);

    std::size_t index = 0, duplication_id = 0;
    // map all way restrictions into access containers
    const auto prepare_way_restriction = [this, &index, &duplication_id, as_duplicated_node](
        const auto &restriction) {

        const auto &way = restriction.AsWayRestriction();
        restriction_starts.insert(
            std::make_pair(std::make_pair(way.in_restriction.via, way.in_restriction.to), index));
        restriction_ends.insert(std::make_pair(
            std::make_pair(way.out_restriction.from, way.out_restriction.via), index));
        via_ways.insert(
            std::make_pair(std::make_pair(way.in_restriction.via, way.out_restriction.via), index));
        ++index;
    };
    std::for_each(restriction_data.begin(), restriction_data.end(), prepare_way_restriction);

    std::size_t offset = 1;
    // the first group starts at 0
    if (!restriction_data.empty())
        duplicated_node_groups.push_back(0);

    auto const add_offset_on_new_groups = [&](auto const &lhs, auto const &rhs) {
        BOOST_ASSERT(rhs == restriction_data[offset]);
        // add a new lower bound for rhs
        if (as_duplicated_node(lhs) != as_duplicated_node(rhs))
            duplicated_node_groups.push_back(offset);
        ++offset;
        return false; // continue until the end
    };
    std::adjacent_find(restriction_data.begin(), restriction_data.end(), add_offset_on_new_groups);
    duplicated_node_groups.push_back(restriction_data.size());

    std::cout << "Node groups:";
    for (auto a : duplicated_node_groups)
        std::cout << " " << a;
    std::cout << std::endl;
}

std::size_t WayRestrictionMap::NumberOfDuplicatedNodes() const
{
    return duplicated_node_groups.size() - 1;
}
std::size_t WayRestrictionMap::DuplicatedNodeID(const std::size_t restriction_id) const
{
    return std::distance(duplicated_node_groups.begin(),
                         std::upper_bound(duplicated_node_groups.begin(),
                                          duplicated_node_groups.end(),
                                          restriction_id)) -
           1;
}

// check if an edge between two nodes is a restricted turn
bool WayRestrictionMap::IsStart(const NodeID from, const NodeID to) const
{
    return restriction_starts.count(std::make_pair(from, to)) > 0;
}
bool WayRestrictionMap::IsEnd(const NodeID from, const NodeID to) const
{
    return restriction_ends.count(std::make_pair(from, to)) > 0;
}
bool WayRestrictionMap::IsViaWay(const NodeID from, const NodeID to) const
{
    return via_ways.count(std::make_pair(from, to)) > 0;
}

std::size_t WayRestrictionMap::Size() const { return via_ways.size(); }

std::vector<std::size_t> WayRestrictionMap::GetIDs(const NodeID from, const NodeID to) const
{
    std::vector<std::size_t> result;
    auto range = via_ways.equal_range(std::make_pair(from, to));
    std::transform(range.first, range.second, std::back_inserter(result), [](auto pair) {
        return pair.second;
    });
    // group by their respective duplicated nodes
    std::sort(result.begin(), result.end());
    return result;
}

TurnRestriction const &WayRestrictionMap::GetRestriction(const std::size_t id) const
{
    return restriction_data[id];
}

std::vector<WayRestrictionMap::ViaWay> WayRestrictionMap::DuplicatedNodeRepresentatives() const
{
    std::vector<ViaWay> result;
    result.reserve(NumberOfDuplicatedNodes());
    std::transform(duplicated_node_groups.begin(),
                   duplicated_node_groups.end() - 1,
                   std::back_inserter(result),
                   [&](auto const representative_id) -> ViaWay {
                       auto &way = restriction_data[representative_id].AsWayRestriction();
                       return {representative_id, way.in_restriction.via, way.out_restriction.via};
                   });
    return result;
}

} // namespace extractor
} // namespace osrm
