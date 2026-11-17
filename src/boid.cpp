#include "boid.hpp"
#include "configs.hpp"
#include "danger.hpp"
#include <memory>
#include <random>
#include <stdexcept>
#include <vector>

namespace sim {

Boid::Boid(int p_id, const std::string& p_breed, float p_w_sep, float p_w_alig,
           float p_w_cohes, Color p_color, float p_max_speed, float p_r_view,
           float p_r_sep, float p_r_fear, float p_fear_factor,
           float p_cone_height, float p_cone_base_r)
{
  this->id          = p_id;
  this->breed       = p_breed;
  this->w_sep       = p_w_sep;
  this->w_alig      = p_w_alig;
  this->w_cohes     = p_w_cohes;
  this->color       = p_color;
  this->max_speed   = p_max_speed;
  this->r_view      = p_r_view;
  this->r_sep       = p_r_sep;
  this->r_fear      = p_r_fear;
  this->fear_factor = p_fear_factor;
  this->cone_height = p_cone_height;
  this->cone_base_r = p_cone_base_r;

  static std::random_device rd;
  static std::default_random_engine eng(rd());

  std::uniform_real_distribution<float> dist_x(-BOX_HALF_X, BOX_HALF_X);
  std::uniform_real_distribution<float> dist_y(-BOX_HALF_Y, BOX_HALF_Y);
  std::uniform_real_distribution<float> dist_z(-BOX_HALF_Z, BOX_HALF_Z);
  this->pos = {dist_x(eng), dist_y(eng), dist_z(eng)};

  std::uniform_real_distribution<float> dist_dir(-1.0f, 1.0f);
  this->vel     = normalize(Vec3{dist_dir(eng), dist_dir(eng), dist_dir(eng)})
                * this->max_speed;
  this->heading = normalize(this->vel);

  if (max_speed <= 0.0f) {
    throw std::runtime_error{"max_speed must be strictly positive"};
  }

  if (r_view < 0.0f || r_sep < 0.0f || r_fear < 0.0f) {
    throw std::runtime_error{"Radii (view, sep, fear) cannot be negative"};
  }
  if (cone_height <= 0.0f || cone_base_r <= 0.0f) {
    throw std::runtime_error{"Cone dimensions must be positive"};
  }
  if (w_sep < 0.0f || w_alig < 0.0f || w_cohes < 0.0f) {
    throw std::runtime_error{"Reynolds weights should not be negative"};
  }
}

void Boid::apply_wrap()
{
  if (this->pos.x < -BOX_HALF_X)
    this->pos.x += 2 * BOX_HALF_X;
  if (this->pos.x > BOX_HALF_X)
    this->pos.x -= 2 * BOX_HALF_X;
  if (this->pos.y < -BOX_HALF_Y)
    this->pos.y += 2 * BOX_HALF_Y;
  if (this->pos.y > BOX_HALF_Y)
    this->pos.y -= 2 * BOX_HALF_Y;
  if (this->pos.z < -BOX_HALF_Z)
    this->pos.z += 2 * BOX_HALF_Z;
  if (this->pos.z > BOX_HALF_Z)
    this->pos.z -= 2 * BOX_HALF_Z;
}

Vec3 Boid::get_vel() const
{
  return vel;
}

Vec3 Boid::get_heading() const
{
  return heading;
}

void Boid::set_vel(const Vec3 new_vel)
{
  this->vel = new_vel;
  clamp_vel();
}

void Boid::increment_vel(const Vec3 dv)
{
  this->vel += dv;
  clamp_vel();
}

Vec3 Boid::get_pos() const
{
  return pos;
}

void Boid::set_pos(const Vec3 new_pos)
{
  this->pos = new_pos;
  apply_wrap();
}

Color Boid::get_color() const
{
  return this->color;
}

const std::string& Boid::get_breed() const
{
  return this->breed;
}

void Boid::increment_pos(const Vec3 dp)
{
  this->pos += dp;
  apply_wrap();
}

float Boid::get_r_fear() const
{
  return r_fear;
}

float Boid::get_fear_factor() const
{
  return fear_factor;
}

float Boid::get_cone_base_r() const
{
  return cone_base_r;
}

float Boid::get_cone_height() const
{
  return cone_height;
}

void Boid::clamp_vel()
{
  float current_speed_sq = norm_sq(this->vel);
  if (current_speed_sq > (this->max_speed * this->max_speed)) {
    float current_speed = std::sqrt(current_speed_sq);
    this->vel           = (this->vel / current_speed) * this->max_speed;
  }
}

Vec3 Boid::steer(Vec3 desired, float max_force) const
{
  // Reynolds steering behavior: steering = desired_velocity - current_velocity
  if (length_sq(desired) <= 0.0f)
    return {0.0f, 0.0f, 0.0f};
  Vec3 target_vel = normalize(desired) * this->max_speed;
  return limit(target_vel - this->vel, max_force);
}

void Boid::evolve(const std::vector<std::unique_ptr<Boid>>& boids,
                  Danger* const walls[6], float dt)
{
  Vec3 sep_sum = {0.0f, 0.0f, 0.0f};
  Vec3 ali_sum = {0.0f, 0.0f, 0.0f};
  Vec3 coh_sum = {0.0f, 0.0f, 0.0f};

  int view_count = 0;
  int sep_count  = 0;

  const float r_view_sq = this->r_view * this->r_view;
  const float r_sep_sq  = this->r_sep * this->r_sep;

  for (const auto& other : boids) {
    if (this->id == other->id)
      continue;

    Vec3 offset = this->pos - other->pos;
    float d_sq  = length_sq(offset);
    if (d_sq <= 0.0f)
      continue;

    if (d_sq < r_sep_sq) {
      float d = std::sqrt(d_sq);
      // Weight the separation force inversely proportional to the distance
      sep_sum += (offset / d) / d;
      ++sep_count;
    }

    if (this->breed == other->breed && d_sq < r_view_sq) {
      ali_sum += other->vel;
      coh_sum += other->pos;
      ++view_count;
    }
  }

  const float max_force = 4.0f * this->max_speed;

  Vec3 sep{0.0f, 0.0f, 0.0f}, ali{0.0f, 0.0f, 0.0f}, coe{0.0f, 0.0f, 0.0f};

  if (sep_count > 0)
    sep = steer(sep_sum, max_force);

  if (view_count > 0) {
    const float inv_count = 1.0f / static_cast<float>(view_count);
    ali                   = steer(ali_sum * inv_count, max_force);
    Vec3 to_centroid      = coh_sum * inv_count - this->pos;
    coe                   = steer(to_centroid, max_force);
  }

  Vec3 danger_steer     = {0.0f, 0.0f, 0.0f};
  const float r_fear_sq = this->r_fear * this->r_fear;

  auto flee = [&](const Danger* dgr) {
    Vec3 closest_pt = dgr->get_closest_point(this->pos);
    float d_sq      = dist_sq(this->pos, closest_pt);
    if (d_sq > 0.0f && d_sq < r_fear_sq) {
      float d = std::sqrt(d_sq);
      // Repulsive force scales inversely with the distance to the threat
      danger_steer += normalize(this->pos - closest_pt)
                    * ((dgr->get_base_threat() * this->fear_factor) / d);
    }
  };

  for (int i = 0; i < 6; ++i)
    if (walls[i])
      flee(walls[i]);

  for (const auto& other : boids) {
    if (this->breed == other->breed)
      continue;
    // Treat boids of different breeds as predators if they implement the Danger
    // interface
    if (const Danger* dgr = dynamic_cast<const Danger*>(other.get()))
      flee(dgr);
  }

  Vec3 acc = sep * (this->w_sep * g_params.w_sep)
           + ali * (this->w_alig * g_params.w_alig)
           + coe * (this->w_cohes * g_params.w_cohes)
           + danger_steer * g_params.w_fear;

  this->pending_acc = limit(acc, max_force);
  (void)dt;
}

void Boid::apply(float dt)
{
  this->increment_vel(this->pending_acc * dt);
  this->increment_pos(this->get_vel() * dt);

  // Frame-rate independent exponential smoothing for a fluid visual rotation
  constexpr float TURN_RATE = 4.0f;
  float alpha               = 1.0f - std::exp(-TURN_RATE * dt);
  Vec3 target_dir           = normalize(this->vel);
  if (length_sq(target_dir) > 0.0f) {
    this->heading = this->heading + (target_dir - this->heading) * alpha;
    this->heading = normalize(this->heading);
  }
}
} // namespace sim
