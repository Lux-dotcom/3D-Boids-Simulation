#ifndef GAME_STARLING_HPP
#define GAME_STARLING_HPP
#include "boid.hpp"

namespace sim {

class Starling : public Boid
{
 public:
  Starling(int p_id);
};

} // namespace sim

#endif // GAME_STARLING_HPP
