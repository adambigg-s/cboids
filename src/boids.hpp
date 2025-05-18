#ifndef BOIDS_H
#define BOIDS_H

#include <cstdlib>
#include <vector>

#include "vector.hpp"

const int BOID_COUNT = 1000;
const int BOID_SCALE = 5;

typedef struct Boid {
    Vec2 position;
    Vec2 velocity;

    static Boid build(Vec2 pos, Vec2 vel) {
        return Boid{
            .position = pos,
            .velocity = vel,
        };
    }

    void move() {
        this->position.add_assign(this->velocity);
    }

    void contain(float xmin, float xmax, float ymin, float ymax) {
        if (this->position.x > xmax) {
            this->velocity.x *= -1;
        } else if (this->position.x < xmin) {
            this->velocity.x *= -1;
        }
        if (this->position.y > ymax) {
            this->velocity.y *= -1;
        } else if (this->position.y < ymin) {
            this->velocity.y *= -1;
        }
    }

    void clamp_speed(float max_speed, float min_speed) {
        float speed = this->velocity.length();
        if (speed > max_speed) {
            this->velocity = this->velocity.normalized().mul(max_speed);
        }
        if (speed < min_speed) {
            this->velocity = this->velocity.normalized().mul(min_speed);
        }
    }
} Boid;

typedef struct BoidParams {
    int vertices;
    int boid_count;
    float max_speed;
    float min_speed;
    float boid_scale;
    float neighbor_distance;
    float separation_distance;
    float cohesion;
    float alignment;
    float separation;
} BoidParams;

typedef struct BoidManager {
    BoidParams params;
    std::vector<Boid> boids;

    void update_boids(float height, float width) {
        for (Boid &boid : this->boids) {
            boid.move();
            boid.contain(0, width, 0, height);
            boid.clamp_speed(this->params.max_speed, this->params.min_speed);
        }
    }

    void cohesion() {
        for (Boid &target : this->boids) {
            Vec2 center_force = Vec2::zeros();
            int counter = 0;
            for (const Boid &other : this->boids) {
                if (&target == &other) {
                    continue;
                }
                Vec2 relative = other.position.sub(target.position);
                if (relative.length() > this->params.neighbor_distance) {
                    continue;
                }

                center_force.add_assign(other.position);
                counter += 1;
            }
            if (counter == 0) {
                continue;
            }

            center_force.div_assign(counter);
            center_force.sub_assign(target.position);
            center_force.mul_assign(this->params.cohesion);

            target.velocity.add_assign(center_force);
        }
    }

    void alignment() {
        for (Boid &target : this->boids) {
            Vec2 direction_force = Vec2::zeros();
            int counter = 0;
            for (const Boid &other : this->boids) {
                if (&target == &other) {
                    continue;
                }
                Vec2 relative = other.position.sub(target.position);
                if (relative.length() > this->params.neighbor_distance) {
                    continue;
                }

                direction_force.add_assign(other.velocity);
                counter += 1;
            }

            if (counter == 0) {
                continue;
            }

            direction_force.div_assign(counter);
            direction_force.sub_assign(target.velocity);
            direction_force.mul_assign(this->params.alignment);

            target.velocity.add_assign(direction_force);
        }
    }

    void separation() {
        for (Boid &target : this->boids) {
            Vec2 force = Vec2::zeros();
            for (const Boid &other : this->boids) {
                if (&target == &other) {
                    continue;
                }
                Vec2 relative = other.position.sub(target.position);
                if (relative.length() > this->params.separation_distance) {
                    continue;
                }

                force.sub_assign(relative);
            }

            force.mul_assign(this->params.separation);

            target.velocity.add_assign(force);
        }
    }
} BoidManager;

typedef struct World {
    float width;
    float height;
    BoidManager data;

    void add_boid() {
        int x = rand() % (int)this->width;
        int y = rand() % (int)this->height;
        float angle = (float)rand() / RAND_MAX * 3.141592 * 2;
        data.boids.push_back(Boid::build(Vec2::build(x, y), Vec2::build(cos(angle), sin(angle))));
    }

    void update() {
        this->data.cohesion();
        this->data.alignment();
        this->data.separation();
        this->data.update_boids(this->height, this->width);

        while (this->data.boids.size() < this->data.params.boid_count) {
            this->add_boid();
        }
        while (this->data.boids.size() > this->data.params.boid_count) {
            this->data.boids.pop_back();
        }
    }
} World;

#endif
