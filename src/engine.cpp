#include "engine.hpp"
#include "configs.hpp"
#include "raygui.h"
#include "wall.hpp"
#include <raylib.h>
#include <rlgl.h>
#include <iostream>

namespace sim {

Vector3 Engine::to_rl(Vec3 v)
{
  return Vector3{v.x, v.y, v.z};
}
void Engine::init_rl()
{
  InitWindow(WINDOW_WIDTH, WINDOW_HEIGHT, "Boid simulation");
  SetTargetFPS(60);

  // Camera init
  camera.position   = {0.0f, -20.0f, BOX_HALF_Z + 350.0f};
  camera.target     = {0.0f, -20.0f, 0.0f};
  camera.up         = {0.0f, 1.0f, 0.0f};
  camera.fovy       = 60.0f;
  camera.projection = CAMERA_PERSPECTIVE;

  // Set far plane distance
  rlSetClipPlanes(RL_CULL_DISTANCE_NEAR, 5000.0);
}

void Engine::build_walls()
{
  dangers[0] = new Wall(Vec3{BOX_HALF_X, 0, 0}, Vec3{-1, 0, 0},
                        WALL_THREAT_FACTOR); // Right
  dangers[1] = new Wall(Vec3{-BOX_HALF_X, 0, 0}, Vec3{1, 0, 0},
                        WALL_THREAT_FACTOR); // Left
  dangers[2] = new Wall(Vec3{0, BOX_HALF_Y, 0}, Vec3{0, -1, 0},
                        WALL_THREAT_FACTOR); // Up
  dangers[3] = new Wall(Vec3{0, -BOX_HALF_Y, 0}, Vec3{0, 1, 0},
                        WALL_THREAT_FACTOR); // Down
  dangers[4] = new Wall(Vec3{0, 0, BOX_HALF_Z}, Vec3{0, 0, -1},
                        WALL_THREAT_FACTOR); // Front
  dangers[5] = new Wall(Vec3{0, 0, -BOX_HALF_Z}, Vec3{0, 0, 1},
                        WALL_THREAT_FACTOR); // Back
}

bool Engine::setup_screen()
{
  const float labelW = 240, w = 180, h = 44, step = 64;
  const float x =
      (static_cast<float>(GetScreenWidth()) - (labelW + 20.0f + w)) / 2.0f
      + labelW + 20.0f;

  GuiSetStyle(DEFAULT, TEXT_SIZE, 22);

  int edit_breed = -1;
  bool start     = false;

  while (!start && !WindowShouldClose()) {
    BeginDrawing();
    ClearBackground(BLACK);

    GuiSetStyle(DEFAULT, TEXT_SIZE, 34);
    GuiLabel({x - labelW - 20, 90, 460, 40}, "SIMULATION SETUP");
    GuiSetStyle(DEFAULT, TEXT_SIZE, 22);
    GuiLabel({x - labelW - 20, 140, 460, 30}, "Number of boids per species:");

    float y = 220;
    struct Row
    {
      const char* name;
      int* value;
    };

    Row rows[] = {
        {"Swallows", &g_spawn.swallow},
        {"Starlings", &g_spawn.starling},
        {"Seagulls", &g_spawn.seagull},
    };

    for (int i = 0; i < 3; ++i) {
      GuiLabel({x - labelW - 20, y + 6, labelW, h}, rows[i].name);
      // GuiValueBox returns 'true' when the user clicks on it to edit.
      // The last parameter enables the "edit mode" only if the current box
      // matches 'edit_breed'.
      if (GuiValueBox({x, y, w, h}, nullptr, rows[i].value, 0, MAX_PER_BREED,
                      edit_breed == i)) {
        // Toggle edit mode: if already editing this one, turn it off (-1), else
        // turn it on (i).
        edit_breed = (edit_breed == i) ? -1 : i;
      }
      y += step;
    }

    GuiCheckBox({x - labelW - 20, y + 6, h - 8, h - 8}, "Camera rotation",
                &g_orbit_camera);
    y += step;

    auto sum = g_spawn.seagull + g_spawn.swallow + g_spawn.starling;
    if (sum > 600) {
      DrawText("Error: Reduce the total amount of birds to less than 600.",
               static_cast<int>(x - labelW - 20), static_cast<int>(y), 20, RED);
      ;

      // Disables all raygui controls making them unclickable.
      GuiDisable();
    }
    y += step;

    GuiSetStyle(DEFAULT, TEXT_SIZE, 28);
    if (GuiButton({x - labelW - 20, y, labelW + 20 + w, h + 12}, "START")) {
      start = true;
    }
    GuiSetStyle(DEFAULT, TEXT_SIZE, 22);

    // Re-enables the UI so it's ready for the next frame.
    GuiEnable();

    EndDrawing();
  }

  // Restore default font size for the in-sim HUD.
  GuiSetStyle(DEFAULT, TEXT_SIZE, 10);

  return start;
}

void Engine::draw()
{
  float dt = GetFrameTime();
  if (g_orbit_camera)
    UpdateCamera(&camera, CAMERA_ORBITAL);

  // Everyone reads same global state
  for (const auto& boid : boids) {
    boid->evolve(boids, dangers, dt);
  }

  // Apply new steerings all at once
  for (const auto& boid : boids) {
    boid->apply(dt);
  }

  BeginDrawing();

  // Light blue
  ClearBackground({150, 170, 200, 255});

  // Switches the rendering context from 2D screen space to 3D world space using
  // the camera's perspective
  BeginMode3D(camera);

  // Draw boundaries
  DrawCubeWires({0, 0, 0}, 2 * BOX_HALF_X, 2 * BOX_HALF_Y, 2 * BOX_HALF_Z,
                DARKGRAY);

  // Draw each boid
  for (const auto& boid : boids) {
    Vec3 direction = boid->get_heading();

    // Cover edge case (target_dir ≈ -heading (boid turn ~180°) and alpha ≈ 0.5)
    if (norm_sq(direction) < 0.5f)
      continue;
    Vector3 topPos =
        to_rl(boid->get_pos() + direction * boid->get_cone_height());
    DrawCylinderEx(to_rl(boid->get_pos()), topPos, boid->get_cone_base_r(),
                   0.0f, 12, boid->get_color());
  }
  // Ends the 3D projection.
  EndMode3D();

  draw_gui();

  EndDrawing();
}

void Engine::draw_gui() const
{
  // 'step' is the vertical distance between one slider and the next.
  const float x = 14, labelW = 72, w = 190, h = 20, step = 30;
  float y = 44;

  const float panelW = labelW + w + 48;
  const float panelH = 44 + step * 4 + 8;
  // Draw the semi-transparent background panel.
  // Raylib's basic drawing functions expect integers, so we cast the floats.
  DrawRectangle(6, 6, static_cast<int>(panelW), static_cast<int>(panelH),
                Fade(BLACK, 0.6f));
  DrawRectangleLines(6, 6, static_cast<int>(panelW), static_cast<int>(panelH),
                     Fade(WHITE, 0.4f));

  GuiSetStyle(DEFAULT, TEXT_SIZE, 14);
  // Unlike Raylib which uses Color structs (r,g,b,a),
  // Raygui styles require colors to be compressed into a single integer.
  GuiSetStyle(DEFAULT, TEXT_COLOR_NORMAL, ColorToInt(RAYWHITE));
  GuiSetStyle(SLIDER, TEXT_COLOR_NORMAL, ColorToInt(RAYWHITE));

  GuiLabel({x, 16, panelW, h}, "SIM PARAMS");
  // It takes a pointer (float* v) to the actual physics parameter so the slider
  // can modify the simulation in real-time.
  auto slider = [&](const char* name, float* v) {
    GuiSlider({x + labelW, y, w, h}, name, TextFormat("%.2f", *v), v, 0.0f,
              3.0f);
    y += step;
  };

  slider("Separation", &g_params.w_sep);
  slider("Alignment", &g_params.w_alig);
  slider("Cohesion", &g_params.w_cohes);
  slider("Fear", &g_params.w_fear);

  GuiSetStyle(DEFAULT, TEXT_SIZE, 10);

  draw_stats();
}

void Engine::draw_stats() const
{
  struct Stats
  {
    int n     = 0;
    float avg = 0, max = 0, order = 0;
    Color col = RAYWHITE;
  };

  auto stats_of = [&](const char* breed) {
    Stats s;
    Vec3 vel_sum{0, 0, 0};
    // vel_sum accumulates normalized headings to compute the flock's Reynolds
    // order
    float speed_sum = 0.0f;
    int moving      = 0;
    for (const auto& boid : boids) {
      if (boid->get_breed() != breed)
        continue;
      if (s.n == 0)
        s.col = boid->get_color();
      ++s.n;
      Vec3 v   = boid->get_vel();
      float sp = norm(v);
      speed_sum += sp;
      if (sp > s.max)
        s.max = sp;
      if (sp > 0.0f) {
        // FLOCKING PHYSICS: Reynolds' "Order" parameter.
        // We sum the pure directions (normalized velocities) of all moving
        // birds.
        vel_sum += normalize(v);
        ++moving;
      }
    }
    if (s.n)
      s.avg = speed_sum / static_cast<float>(s.n);
    // If all birds fly in the exact same direction, their normalized vectors
    // perfectly add up, so norm(vel_sum) will equal 'moving', making the Order
    // exactly 1.0 (perfect alignment). If they fly randomly, the vectors cancel
    // each other out, making the Order approach 0.0 (chaos).
    s.order = moving ? norm(vel_sum) / static_cast<float>(moving) : 0.0f;
    return s;
  };

  const int pw = 190, ph = 96;
  auto panel = [&](const char* name, const char* breed, int px, int py) {
    Stats s      = stats_of(breed);
    Color col    = s.col;
    const int hh = 26; // Header height

    DrawRectangle(px, py, pw, ph, Fade(BLACK, 0.6f));
    DrawRectangle(px, py, pw, hh, col);
    DrawRectangleLines(px, py, pw, ph, col);

    // Standard formula to calculate relative luminance to ensure text is always
    // readable
    float lum = (0.299f * col.r + 0.587f * col.g + 0.114f * col.b) / 255.0f;
    Color head_txt = lum > 0.5f ? BLACK : RAYWHITE;

    int tx = px + 8, ty = py + 5;
    DrawText(TextFormat("%s (%d)", name, s.n), tx, ty, 18, head_txt);
    ty += hh + 4;
    DrawText(TextFormat("Avg speed %.1f", s.avg), tx, ty, 14, RAYWHITE);
    ty += 18;
    DrawText(TextFormat("Max speed %.1f", s.max), tx, ty, 14, RAYWHITE);
    ty += 18;
    DrawText(TextFormat("Order     %.2f", s.order), tx, ty, 14, RAYWHITE);
  };

  // Calculate dynamic corner positions using screen width (W), panel width
  // (pw), and margin (m)
  const int W = GetScreenWidth(), H = GetScreenHeight(), m = 10;

  // Dynamic FPS color
  DrawText(TextFormat("FPS %d", GetFPS()), W / 2 - 40, m, 18,
           GetFPS() >= 55 ? GREEN : (GetFPS() >= 30 ? YELLOW : RED));

  // Draw the panels pinned to the corners of the window
  panel("Swallows", "swallow", W - pw - m, m);          // TOP RIGHT
  panel("Starlings", "starling", m, H - ph - m);        // BOTTOM LEFT
  panel("Seagulls", "seagull", W - pw - m, H - ph - m); // BOTTOM RIGHT
}

Engine::~Engine()
{
  for (Danger* d : dangers)
    delete d;
  CloseWindow();
}

} // namespace sim
