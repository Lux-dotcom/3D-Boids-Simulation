#include "swallow.hpp"

namespace sim {

Swallow::Swallow(int p_id)
    : Boid(p_id, "swallow", 1, 1, 1, BLACK, 105, 60, 30, 60, 1, 16.0f, 5.0f)
{}

} // namespace sim
