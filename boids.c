#include <stdio.h>
#include <raylib.h>
#include <raymath.h>
#include <stdbool.h>

#define UT_IMPLEMENTATION
#include <utils.h>

#define DARKER_GRAY ((Color) {18, 18, 18, 0xFF})

typedef struct {
  Vector2 center;
  Vector2 direction;
  float speed;
} Boid;

typedef ut_da_declare(Boid) Boids;

#define TRIANGLE_SIZE 20.0f
#define BOID_RADIUS (TRIANGLE_SIZE * 6)

Vector2 random_direction(void)
{
  Vector2 d = {
    .x = (float) GetRandomValue(-1 * GetScreenWidth(), GetScreenWidth()),
    .y = (float) GetRandomValue(-1 * GetScreenHeight(), GetScreenHeight())
  };
  return Vector2Normalize(d);
}

void avoid_walls(Boid *b)
{
    float base_distance = 50.0f;
    float lookahead_factor = 5.0f;
    float steer_base = 0.155f;

    float lookahead = base_distance + b->speed * lookahead_factor;
    float steer_strength = steer_base * (b->speed / 3.0f);

    Vector2 steer = {0};

    if (b->center.x < lookahead) steer.x += steer_strength;
    else if (b->center.x > GetScreenWidth() - lookahead) steer.x -= steer_strength;

    if (b->center.y < lookahead) steer.y += steer_strength;
    else if (b->center.y > GetScreenHeight() - lookahead) steer.y -= steer_strength;

    if (Vector2Length(steer) > 0)
        b->direction = Vector2Normalize(Vector2Add(b->direction, steer));
}

void steer_boid_towards_direction(Boid *b, Vector2 d)
{
  float steer_factor = 0.1f;
  b->direction = Vector2Normalize(Vector2Lerp(b->direction, d, steer_factor));
}

Boids local_neighbours(Boids *bs, size_t index_b)
{
  Boids neighbours = {0};
  for (size_t i = 0; i < bs->count; i++) {
    if (i == index_b) continue;
    if (CheckCollisionCircles(bs->items[index_b].center, BOID_RADIUS, bs->items[i].center, BOID_RADIUS))
      ut_da_push(&neighbours, bs->items[i]);
  }
  return neighbours;
}

void alignment(Boids *bs)
{
  // Steer to general direction of local boids
  // at least 3 boids
  if (bs->count < 2) return;
  Vector2 d = {0};
  for (size_t i = 0; i < bs->count; i++){
    Boids lbs = local_neighbours(bs, i);

    // at least 4 boids
    if (lbs.count < 2) {
      ut_da_free(&lbs);
      continue;
    }
    d = Vector2Zero();

    for (size_t j = 0; j < lbs.count; j++)
      d = Vector2Add(d, lbs.items[j].direction);
    ut_da_free(&lbs);

    d = Vector2Normalize(d);
    steer_boid_towards_direction(&bs->items[i], d);
  }
}

void separation(Boids *bs)
{
  // Separate from most closest local neighbour
  // at least 3 boids
  if (bs->count < 2) return;
  for (size_t i = 0; i < bs->count; i++) {
    Boids lbs = local_neighbours(bs, i);

    if (lbs.count < 2) {
      ut_da_free(&lbs);
      continue;
    }

    float min_distance_threshold = BOID_RADIUS / 3;
    Vector2 d = Vector2Zero();

    Boids closest_neighbours = {0};

    for (size_t j = 0; j < lbs.count; j++) {
      float distance = Vector2Distance(bs->items[i].center, lbs.items[j].center);
      if ((distance < min_distance_threshold) && distance > 0) {
        ut_da_push(&closest_neighbours, lbs.items[j]);
        Vector2 diff = Vector2Subtract(bs->items[i].center, lbs.items[j].center);
        diff = Vector2Normalize(diff);

        // Stronger repulsion if closer — inverse distance scaling
        float strength = (min_distance_threshold - distance) / min_distance_threshold;
        diff = Vector2Scale(diff, strength);

        d = Vector2Add(d, diff);
      }
    }

    if (closest_neighbours.count > 0) {
      d = Vector2Scale(d, 1.0f / closest_neighbours.count);
      d = Vector2Normalize(d);

      // steer d — stronger if moving fast
      float steer_factor = 0.1f * (bs->items[i].speed / 3.0f);
      bs->items[i].direction = Vector2Normalize(
          Vector2Lerp(bs->items[i].direction, d, steer_factor)
          );
    }

    ut_da_free(&closest_neighbours);
    ut_da_free(&lbs);
  }
}

void cohesion(Boids *bs)
{
    if (bs->count < 2) return;

    for (size_t i = 0; i < bs->count; i++) {
        Boids lbs = local_neighbours(bs, i);
        if (lbs.count < 1) {
            ut_da_free(&lbs);
            continue;
        }

        // Compute average position of local neighbors
        Vector2 average_pos = Vector2Zero();
        for (size_t j = 0; j < lbs.count; j++) {
            average_pos = Vector2Add(average_pos, lbs.items[j].center);
        }
        average_pos = Vector2Scale(average_pos, 1.0f / lbs.count);

        // Desired direction toward the local center of mass
        Vector2 desired = Vector2Subtract(average_pos, bs->items[i].center);
        desired = Vector2Normalize(desired);

        // Steer smoothly toward that direction
        float steer_factor = 0.05f * (bs->items[i].speed / 3.0f);
        bs->items[i].direction = Vector2Normalize(
            Vector2Lerp(bs->items[i].direction, desired, steer_factor)
        );

        ut_da_free(&lbs);
    }
}

int main(void)
{
  Boids bs = {0};

  int ratio_width = 16;
  int ratio_height = 9;
  int factor = 100;
  InitWindow(factor * ratio_width, factor * ratio_height, "Boids");
  SetTargetFPS(60);

  bool pause_game = false;
  bool draw_circles = false;
  while(! WindowShouldClose()) {
    ClearBackground(DARKER_GRAY);

    if (!pause_game) {
      // Create Boid
      if (IsMouseButtonPressed(MOUSE_BUTTON_RIGHT) &&
         (GetMousePosition().x > TRIANGLE_SIZE) &&
         (GetMousePosition().x < GetScreenWidth() - TRIANGLE_SIZE) &&
         (GetMousePosition().y > TRIANGLE_SIZE) &&
         (GetMousePosition().y < GetScreenHeight() - TRIANGLE_SIZE)
         )
        ut_da_push(&bs, ((Boid){
              .center    = GetMousePosition(),
              .direction = random_direction(),
              .speed     = (float) GetRandomValue(3, 5)
              }));

      // Add rules
      separation(&bs);
      alignment(&bs);
      cohesion(&bs);
      for (size_t i = 0; i < bs.count; i++) {
        // steer_to_mouse(&b.items[i]);
        avoid_walls(&bs.items[i]);
        Vector2 velocity = Vector2Scale(bs.items[i].direction, bs.items[i].speed);
        bs.items[i].center = Vector2Add(bs.items[i].center, velocity);
      }
    }

    if (IsKeyPressed(KEY_SPACE)) pause_game = !pause_game;
    if (IsKeyPressed(KEY_F)) draw_circles = !draw_circles;
    BeginDrawing();
    {
      for (size_t i = 0; i < bs.count; i++) {
        float angle = atan2f(bs.items[i].direction.y, bs.items[i].direction.x) + PI / 2;
        Vector2 v1 = Vector2Rotate((Vector2){-TRIANGLE_SIZE/2,  1.5 * TRIANGLE_SIZE / 3}, angle);
        Vector2 v2 = Vector2Rotate((Vector2){ TRIANGLE_SIZE/2,  1.5 * TRIANGLE_SIZE / 3}, angle);
        Vector2 v3 = Vector2Rotate((Vector2){               0,  1.5 * TRIANGLE_SIZE * -2 / 3}, angle);

        v1 = Vector2Add(v1, bs.items[i].center);
        v2 = Vector2Add(v2, bs.items[i].center);
        v3 = Vector2Add(v3, bs.items[i].center);
        DrawTriangle(v1, v2, v3, RED);

        if (draw_circles)
          DrawCircleLinesV(bs.items[i].center, BOID_RADIUS, GREEN);
      }
    }
    EndDrawing();
  }

  CloseWindow();
  ut_da_free(&bs);
  return 0;
}
