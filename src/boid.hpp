#ifndef BOID_HPP
#define BOID_HPP
#include "vec3.hpp"
#include <raylib.h>
#include <memory>
#include <string>
#include <vector>

namespace sim {

class Danger;

class Boid
{
  int id;
  std::string breed;
  void clamp_vel();
  void apply_wrap(); // Applies toroidal boundary conditions (wrap-around)
  Vec3 steer(Vec3 desired, float max_force) const;

 protected:
  Vec3 pos;
  Vec3 vel;
  Vec3 heading{0.0f, 0.0f, 0.0f};
  Vec3 pending_acc{0.0f, 0.0f, 0.0f};
  float max_speed, r_view, r_sep, r_fear, fear_factor;
  Color color;
  float cone_height, cone_base_r;
  float w_sep, w_alig, w_cohes;

 public:
  Boid(int p_id, const std::string& p_breed, float p_w_sep, float p_w_alig,
       float p_w_cohes, Color p_color, float p_max_speed, float p_r_view,
       float p_r_sep, float p_r_fear, float p_fear_factor, float p_cone_height,
       float p_cone_base_r);

  virtual ~Boid() = default;

  void set_vel(const Vec3 new_vel);

  void increment_vel(const Vec3 dv);

  void set_pos(const Vec3 new_pos);

  void increment_pos(const Vec3 dp);

  Vec3 get_vel() const;

  Vec3 get_heading() const;

  Vec3 get_pos() const;

  Color get_color() const;

  const std::string& get_breed() const;

  float get_r_fear() const;

  float get_fear_factor() const;

  float get_cone_height() const;

  float get_cone_base_r() const;

  void evolve(const std::vector<std::unique_ptr<Boid>>& boids,
              Danger* const walls[6], float dt);

  void apply(float dt);
};

} // namespace sim

#endif
