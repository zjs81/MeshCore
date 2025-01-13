#pragma once

#include <Mesh.h>

namespace mesh {

/**
 * An abstraction of the data tables needed to be maintained, for the routing engine.
*/
class MeshTables {
public:
  virtual bool hasForwarded(const uint8_t* packet_hash) const = 0;
  virtual void setHasForwarded(const uint8_t* packet_hash) = 0;
};

}