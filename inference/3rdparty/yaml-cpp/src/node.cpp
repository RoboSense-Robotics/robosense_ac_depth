#include "yaml-cpp/node/node.h"
#include "nodebuilder.h"
#include "nodeevents.h"

namespace RS_YAML {
Node Clone(const Node& node) {
  NodeEvents events(node);
  NodeBuilder builder;
  events.Emit(builder);
  return builder.Root();
}
}  // namespace RS_YAML
