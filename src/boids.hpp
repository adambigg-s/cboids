#ifndef BOIDS_H
#define BOIDS_H

#include <cstdlib>
#include <vector>

#include "vector.hpp"

const int BOID_COUNT = 3000;
const int BOID_SCALE = 10;

typedef struct Boid {
    Vec2 pos;
    Vec2 vel;

    static Boid build(Vec2 pos, Vec2 vel) {
        return Boid{
            .pos = pos,
            .vel = vel,
        };
    }

    void move() {
        this->pos.add_assign(this->vel);
    }

    void contain(float xmin, float xmax, float ymin, float ymax) {
        if (this->pos.x > xmax) {
            this->pos.x -= xmin;
        } else if (this->pos.x < xmin) {
            this->pos.x += xmax;
        }
        if (this->pos.y > ymax) {
            this->pos.y -= ymin;
        } else if (this->pos.y < ymin) {
            this->pos.y += ymax;
        }
    }
} Boid;

typedef struct BoidManager {
    std::vector<Boid> boids;

    void update_boids(float height, float width) {
        for (Boid &boid : this->boids) {
            boid.move();
            boid.contain(0, width, 0, height);
        }
    }

    void cohese_boids() {
        for (Boid &target : this->boids) {
            Vec2 center = Vec2::zeros();
            int counter = 0;
            for (const Boid &other : this->boids) {
                if (&target == &other) {
                    continue;
                }
                Vec2 relative = other.pos.sub(target.pos);
                if (relative.length() > 100) {
                    continue;
                }

                center.add_assign(other.pos);
                counter += 1;
            }

            if (counter == 0) {
                continue;
            }

            center.div_assign(counter);
            Vec2 delta = center.sub(target.pos);
            delta.div_assign(10000);

            target.vel.add_assign(delta);
        }
    }

    void align_boids() {
        for (Boid &target : this->boids) {
            Vec2 direc = Vec2::zeros();
            int counter = 0;
            for (const Boid &other : this->boids) {
                if (&target == &other) {
                    continue;
                }
                Vec2 relative = other.pos.sub(target.pos);
                if (relative.length() > 100) {
                    continue;
                }

                direc.add_assign(other.vel);
                counter += 1;
            }

            if (counter == 0) {
                continue;
            }

            Vec2 delta = direc.div(counter);
            delta.sub_assign(target.vel);
            delta.div_assign(100);

            target.vel.add_assign(delta);
        }
    }

    void separate_boids() {
        for (Boid &target : this->boids) {
            Vec2 force = Vec2::zeros();
            for (const Boid &other : this->boids) {
                if (&target == &other) {
                    continue;
                }
                Vec2 relative = other.pos.sub(target.pos);
                if (relative.length() > 100) {
                    continue;
                }

                force.sub_assign(relative);
            }

            Vec2 delta = force.div(10000);
            target.vel.add_assign(delta);
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
        this->data.cohese_boids();
        this->data.align_boids();
        this->data.separate_boids();
        this->data.update_boids(this->height, this->width);
    }
} World;

#endif
