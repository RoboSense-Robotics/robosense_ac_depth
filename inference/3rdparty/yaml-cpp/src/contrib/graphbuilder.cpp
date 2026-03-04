#include "graphbuilderadapter.h"

#include "yaml-cpp/parser.h"  // IWYU pragma: keep

namespace RS_YAML {
class GraphBuilderInterface;

void* BuildGraphOfNextDocument(Parser& parser,
                               GraphBuilderInterface& graphBuilder) {
  GraphBuilderAdapter eventHandler(graphBuilder);
  if (parser.HandleNextDocument(eventHandler)) {
    return eventHandler.RootNode();
  }
  return nullptr;
}
}  // namespace RS_YAML
