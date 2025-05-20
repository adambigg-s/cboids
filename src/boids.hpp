#ifndef BOIDS_H
#define BOIDS_H

#define PI 3.141592
#define TAU PI * 2

#include <cstdlib>
#include <vector>

#include "vector.hpp"

typedef struct BoundingBox {
    float xmin;
    float xmax;
    float ymin;
    float ymax;

    Vec2 box_wrapped_postion(Vec2 *to, Vec2 *from) {
        float x_range = this->xmax - this->xmin;
        float y_range = this->ymax - this->ymin;

        Vec2 direct = to->sub(*from);

        if (direct.x > x_range / 2) {
            direct.x -= x_range;
        } else if (direct.x < -x_range / 2) {
            direct.x += x_range;
        }

        if (direct.y > y_range / 2) {
            direct.y -= y_range;
        } else if (direct.y < -y_range / 2) {
            direct.y += y_range;
        }

        return direct;
    }
} BoundingBox;

typedef struct Boid {
    Vec2 position;
    Vec2 velocity;
    Vec2 acceleration;

    static Boid build(Vec2 pos, Vec2 vel) {
        return Boid{
            .position = pos,
            .velocity = vel,
            .acceleration = Vec2::zeros(),
        };
    }

    void move() {
        this->position.add_assign(this->velocity);
    }

    void integrate() {
        this->velocity.add_assign(this->acceleration);
    }

    void reset_forces() {
        this->acceleration.mul_assign(0);
    }

    void contain(BoundingBox *bounds) {
        if (this->position.x > bounds->xmax) {
            this->position.x = bounds->xmin;
        } else if (this->position.x < bounds->xmin) {
            this->position.x = bounds->xmax;
        }
        if (this->position.y > bounds->ymax) {
            this->position.y = bounds->ymin;
        } else if (this->position.y < bounds->ymin) {
            this->position.y = bounds->ymax;
        }
    }

    void clamp_speed(float max_speed, float min_speed) {
        float speed = this->velocity.length();
        if (speed > max_speed) {
            this->velocity.mul_assign(max_speed / speed);
        }
        if (speed < min_speed) {
            this->velocity.mul_assign(min_speed / speed);
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

    void update_boids(BoundingBox *bounds) {
        for (Boid &boid : this->boids) {
            boid.integrate();
            boid.reset_forces();
            boid.clamp_speed(this->params.max_speed, this->params.min_speed);
            boid.contain(bounds);
            boid.move();
        }
    }

    void cohesion(BoundingBox *bounds) {
        for (Boid &target : this->boids) {
            Vec2 center_force = Vec2::zeros();
            int counter = 0;

            for (const Boid &other : this->boids) {
                if (&target == &other) {
                    continue;
                }

                Vec2 target_position = target.position;
                Vec2 other_position = other.position;
                other_position = bounds->box_wrapped_postion(&target_position, &other_position);

                Vec2 relative = other_position.sub(target_position);
                if (relative.length() > this->params.neighbor_distance) {
                    continue;
                }

                center_force.add_assign(other_position);
                counter += 1;
            }
            if (counter == 0) {
                continue;
            }

            center_force.div_assign(counter);
            center_force.sub_assign(target.position);
            center_force.mul_assign(this->params.cohesion);

            target.acceleration.add_assign(center_force);
        }
    }

    void alignment(BoundingBox *bounds) {
        for (Boid &target : this->boids) {
            Vec2 direction_force = Vec2::zeros();
            int counter = 0;
            for (const Boid &other : this->boids) {
                if (&target == &other) {
                    continue;
                }

                Vec2 target_position = target.position;
                Vec2 other_position = other.position;
                other_position = bounds->box_wrapped_postion(&target_position, &other_position);

                Vec2 relative = other_position.sub(target_position);
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

            target.acceleration.add_assign(direction_force);
        }
    }

    void separation(BoundingBox *bounds) {
        for (Boid &target : this->boids) {
            Vec2 force = Vec2::zeros();
            for (const Boid &other : this->boids) {
                if (&target == &other) {
                    continue;
                }

                Vec2 target_position = target.position;
                Vec2 other_position = other.position;
                other_position = bounds->box_wrapped_postion(&target_position, &other_position);

                Vec2 relative = other_position.sub(target_position);
                if (relative.length() > this->params.separation_distance) {
                    continue;
                }

                force.sub_assign(relative);
            }

            force.mul_assign(this->params.separation);

            target.acceleration.add_assign(force);
        }
    }
} BoidManager;

typedef struct World {
    BoundingBox bounds;
    BoidManager data;

    void add_boid() {
        int x = rand() % (int)this->bounds.xmax;
        int y = rand() % (int)this->bounds.ymax;
        float angle = (float)rand() / RAND_MAX * TAU;
        data.boids.push_back(Boid::build(
            Vec2::build(x, y), Vec2::build(cos(angle), sin(angle)).mul(this->data.params.max_speed)));
    }

    void update() {
        this->data.cohesion(&this->bounds);
        this->data.alignment(&this->bounds);
        this->data.separation(&this->bounds);
        this->data.update_boids(&this->bounds);

        while (this->data.boids.size() < this->data.params.boid_count) {
            this->add_boid();
        }
        while (this->data.boids.size() > this->data.params.boid_count) {
            this->data.boids.pop_back();
        }
    }
} World;

#endif
