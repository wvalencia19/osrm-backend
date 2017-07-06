functions = require('lib/profile_v2')

function setup()
return {
  node_tags_requiring_processing = {
    'barrier'
  }
}
end

function process_node(profile, node, result)
  print ('process_node ' .. node:id())
end

function process_way(profile, way, result)
  result.name = way:get_value_by_key('name')
  result.weight = 10
  result.forward_mode = mode.driving
  result.backward_mode = mode.driving
  result.forward_speed = 36
  result.backward_speed = 36
end

return {
  setup = setup,
  process_node = process_node,
  process_way = process_way
}
