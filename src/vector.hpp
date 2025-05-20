#ifndef VECTOR_H
#define VECTOR_H

#include <cmath>

typedef struct Vec2 {
    float x;
    float y;

    static Vec2 build(float x, float y) {
        return Vec2{.x = x, .y = y};
    }

    static Vec2 zeros() {
        return Vec2::build(0, 0);
    }

    Vec2 add(Vec2 other) const {
        return Vec2::build(this->x + other.x, this->y + other.y);
    }

    Vec2 sub(Vec2 other) const {
        return Vec2::build(this->x - other.x, this->y - other.y);
    }

    Vec2 mul(float value) const {
        return Vec2::build(this->x * value, this->y * value);
    }

    Vec2 div(float value) const {
        return Vec2::build(this->x / value, this->y / value);
    }

    void add_assign(Vec2 other) {
        this->x += other.x;
        this->y += other.y;
    }

    void sub_assign(Vec2 other) {
        this->x -= other.x;
        this->y -= other.y;
    }

    void mul_assign(float value) {
        this->x *= value;
        this->y *= value;
    }

    void div_assign(float value) {
        this->x /= value;
        this->y /= value;
    }

    float inner_product(Vec2 other) {
        return this->x * other.x + this->y * other.y;
    }

    float length() {
        return sqrt(this->inner_product(*this));
    }

    Vec2 normalized() {
        return this->div(this->length());
    }

    float angle(Vec2 other) {
        return acos(this->inner_product(other) / (this->length() * other.length()));
    }
} Vec2;

#endif
