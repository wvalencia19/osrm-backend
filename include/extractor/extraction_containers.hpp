#ifndef EXTRACTION_CONTAINERS_HPP
#define EXTRACTION_CONTAINERS_HPP

#include "extractor/first_and_last_segment_of_way.hpp"
#include "extractor/internal_extractor_edge.hpp"
#include "extractor/query_node.hpp"
#include "extractor/restriction.hpp"
#include "extractor/scripting_environment.hpp"

#include "storage/io.hpp"

namespace osrm
{
namespace extractor
{

/**
 * Uses  memory containers to store all the data that
 * is collected by the extractor callbacks.
 *
 * The data is the filtered, aggregated and finally written to disk.
 */
class ExtractionContainers
{
    void PrepareNodes();
    void PrepareRestrictions();
    void PrepareEdges(ScriptingEnvironment &scripting_environment);

    void WriteNodes(storage::io::FileWriter &file_out) const;
    void WriteConditionalRestrictions(const std::string &restrictions_file_name);
    void WriteEdges(storage::io::FileWriter &file_out) const;
    void WriteCharData(const std::string &file_name);

  public:
    using NodeIDVector = std::vector<OSMNodeID>;
    using NodeVector = std::vector<QueryNode>;
    using EdgeVector = std::vector<InternalExtractorEdge>;
    using WayIDStartEndVector = std::vector<FirstAndLastSegmentOfWay>;
    using NameCharData = std::vector<unsigned char>;
    using NameOffsets = std::vector<unsigned>;

    std::vector<OSMNodeID> barrier_nodes;
    std::vector<OSMNodeID> traffic_lights;
    NodeIDVector used_node_id_list;
    NodeVector all_nodes_list;
    EdgeVector all_edges_list;
    NameCharData name_char_data;
    NameOffsets name_offsets;
    // an adjacency array containing all turn lane masks
    WayIDStartEndVector way_start_end_id_list;

    unsigned max_internal_node_id;

    // list of restrictions before we transform them into the output types
    std::vector<InputConditionalTurnRestriction> restrictions_list;

    // turn restrictions split into conditional and unconditional turn restrictions
    std::vector<ConditionalTurnRestriction> conditional_turn_restrictions;
    std::vector<TurnRestriction> unconditional_turn_restrictions;

    ExtractionContainers();

    void PrepareData(ScriptingEnvironment &scripting_environment,
                     const std::string &output_file_name,
                     const std::string &restrictions_file_name,
                     const std::string &names_file_name);
};
}
}

#endif /* EXTRACTION_CONTAINERS_HPP */
