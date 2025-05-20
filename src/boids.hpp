#ifndef BOIDS_H
#define BOIDS_H

#define PI 3.141592
#define TAU PI * 2

#include <vector>

#include "vector.hpp"

typedef struct VectorData {
    Vec2 vector;
    float length;

    static VectorData build(Vec2 vector) {
        return VectorData{.vector = vector, .length = vector.length()};
    }
} VectorData;

typedef struct BoundingBox {
    float xmin;
    float xmax;
    float ymin;
    float ymax;

    Vec2 box_wrapped_postion(const Vec2 *from, const Vec2 *to) {
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
    float peripheral_angle;
    float wall_distance;
    float wall_strength;
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
        float epsilon = 1e-4;
        if (this->position.x > bounds->xmax) {
            this->position.x = bounds->xmax - epsilon;
            this->velocity.x *= -1;
        } else if (this->position.x < bounds->xmin) {
            this->position.x = bounds->xmin + epsilon;
            this->velocity.x *= -1;
        }
        if (this->position.y > bounds->ymax) {
            this->position.y = bounds->ymax - epsilon;
            this->velocity.y *= -1;
        } else if (this->position.y < bounds->ymin) {
            this->position.y = bounds->ymin + epsilon;
            this->velocity.y *= -1;
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

    void avoid_walls(BoundingBox *bounds, BoidParams *params) {
        Vec2 repulsion = Vec2::zeros();
        if (this->position.x < bounds->xmin + params->wall_distance) {
            float distance = this->position.x - bounds->xmin;
            repulsion.x += params->wall_strength / (distance * distance);
        }
        if (this->position.x > bounds->xmax - params->wall_distance) {
            float distance = bounds->xmax - this->position.x;
            repulsion.x -= params->wall_strength / (distance * distance);
        }
        if (this->position.y < bounds->ymin + params->wall_distance) {
            float distance = this->position.y - bounds->ymin;
            repulsion.y += params->wall_strength / (distance * distance);
        }
        if (this->position.y > bounds->ymax - params->wall_distance) {
            float distance = bounds->ymax - this->position.y;
            repulsion.y -= params->wall_strength / (distance * distance);
        }

        this->acceleration.add_assign(repulsion);
    }

    void cohesion(VectorData *pointer, Vec2 *force, int *counter, BoidParams *params) {
        if (pointer->length > params->neighbor_distance) {
            return;
        }

        *counter += 1;
        force->add_assign(pointer->vector);
    }

    void alignment(VectorData *pointer, Vec2 *force, int *counter, BoidParams *params, const Boid *other) {
        if (pointer->length > params->neighbor_distance) {
            return;
        }

        *counter += 1;
        force->add_assign(other->velocity);
    }

    void separation(VectorData *pointer, Vec2 *force, BoidParams *params) {
        if (pointer->length > params->separation_distance || pointer->length < 1e-4) {
            return;
        }

        Vec2 repulsion = pointer->vector;
        float inv = -1 / (pointer->length * pointer->length);
        repulsion.mul_assign(inv);

        force->add_assign(repulsion);
    }
} Boid;

typedef struct BoidManager {
    BoidParams params;
    std::vector<Boid> boids;

    void update_boids(BoundingBox *bounds, float delta_time) {
        for (Boid &target : this->boids) {

            Vec2 cohesion_force = Vec2::zeros();
            Vec2 alignment_force = Vec2::zeros();
            Vec2 separation_force = Vec2::zeros();

            int cohesion_count = 0;
            int alignment_count = 0;

            for (const Boid &other : this->boids) {
                if (&target == &other) {
                    continue;
                }

                Vec2 relative = other.position.sub(target.position);
                if (target.velocity.angle(relative) > params.peripheral_angle) {
                    continue;
                }
                VectorData pointer = VectorData::build(relative);

                target.cohesion(&pointer, &cohesion_force, &cohesion_count, &this->params);
                target.alignment(&pointer, &alignment_force, &alignment_count, &this->params, &other);
                target.separation(&pointer, &separation_force, &this->params);
            }

            if (cohesion_count > 0) {
                cohesion_force.div_assign(cohesion_count);
                cohesion_force.mul_assign(this->params.cohesion);
                target.acceleration.add_assign(cohesion_force);
            }

            if (alignment_count > 0) {
                alignment_force.div_assign(alignment_count);
                alignment_force.mul_assign(this->params.alignment);
                target.acceleration.add_assign(alignment_force);
            }

            separation_force.mul_assign(this->params.separation);
            target.acceleration.add_assign(separation_force);
        }

        for (Boid &boid : this->boids) {
            boid.avoid_walls(bounds, &this->params);
            boid.integrate(delta_time);
            boid.clamp_speed(this->params.max_speed, this->params.min_speed);
            boid.move(delta_time);
            boid.contain(bounds);
            boid.reset_forces();
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
