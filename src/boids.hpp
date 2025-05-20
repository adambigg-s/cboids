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

    Vec2 box_wrapped_postion(const Vec2 *to, const Vec2 *from) {
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

    void move(float delta_time) {
        this->position.add_assign(this->velocity.mul(delta_time));
    }

    void integrate(float delta_time) {
        this->velocity.add_assign(this->acceleration.mul(delta_time));
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

    void cohesion(Vec2 *pointer, Vec2 *cohesion, int *counter, BoidParams *params) {
        float distance = pointer->length();

        if (distance > params->neighbor_distance) {
            return;
        }

        *counter += 1;
        cohesion->add_assign(*pointer);
    }

    void alignment(Vec2 *pointer, Vec2 *alignment, int *counter, BoidParams *params, const Boid *other) {
        float distance = pointer->length();

        if (distance > params->neighbor_distance) {
            return;
        }

        *counter += 1;
        alignment->add_assign(other->velocity);
    }

    void separation(Vec2 *pointer, Vec2 *separation, BoidParams *params) {
        float distance = pointer->length();

        if (distance > params->separation_distance) {
            return;
        }

        Vec2 repulsion = pointer->normalized();
        float inv = -1 / distance;
        repulsion.mul_assign(inv);

        separation->add_assign(repulsion);
    }
} Boid;

typedef struct BoidManager {
    BoidParams params;
    std::vector<Boid> boids;

    void update_boids(BoundingBox *bounds, float delta_time) {
        for (Boid &target : this->boids) {

            Vec2 cohesion = Vec2::zeros();
            Vec2 alignment = Vec2::zeros();
            Vec2 separation = Vec2::zeros();

            int cohesion_count = 0;
            int alignment_count = 0;

            for (const Boid &other : this->boids) {
                if (&target == &other) {
                    continue;
                }

                Vec2 relative_position = bounds->box_wrapped_postion(&target.position, &other.position);

                target.cohesion(&relative_position, &cohesion, &cohesion_count, &this->params);
                target.alignment(&relative_position, &alignment, &alignment_count, &this->params, &other);
                target.separation(&relative_position, &separation, &this->params);
            }

            if (cohesion_count > 0) {
                cohesion.div_assign(cohesion_count);
                cohesion.mul_assign(this->params.cohesion);
                target.acceleration.add_assign(cohesion);
            }

            if (alignment_count > 0) {
                alignment.div_assign(alignment_count);
                alignment.mul_assign(this->params.alignment);
                target.acceleration.add_assign(alignment);
            }

            separation.mul_assign(this->params.separation);
            target.acceleration.add_assign(separation);
        }

        for (Boid &boid : this->boids) {
            boid.integrate(delta_time);
            boid.reset_forces();
            boid.clamp_speed(this->params.max_speed, this->params.min_speed);
            boid.contain(bounds);
            boid.move(delta_time);
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

    void update(float delta_time) {
        this->data.update_boids(&this->bounds, delta_time);

        while (this->data.boids.size() < this->data.params.boid_count) {
            this->add_boid();
        }
        while (this->data.boids.size() > this->data.params.boid_count) {
            this->data.boids.pop_back();
        }
    }
} World;

#endif
