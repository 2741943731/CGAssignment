#pragma once
#ifndef BOX_H
#define BOX_H

#include "rtweekend.h"
#include "xy_rect.h"
#include "hittable_list.h"

class box : public hittable {
public:
    box() {}
    box(const vec3& p0, const vec3& p1, shared_ptr<material> ptr);

    box(const vec3& p0, const vec3& p1, shared_ptr<material> front_ptr, shared_ptr<material> lateral_ptr, shared_ptr<material> top_ptr);

    virtual bool hit(const ray& r, double t0, double t1, hit_record& rec) const;

    virtual bool bounding_box(double t0, double t1, aabb& output_box) const {
        output_box = aabb(box_min, box_max);
        return true;
    }

public:
    vec3 box_min;
    vec3 box_max;
    hittable_list sides;
};

box::box(const vec3& p0, const vec3& p1, shared_ptr<material> ptr) {
    box_min = p0;
    box_max = p1;

    sides.add(make_shared<xy_rect>(p0.x(), p1.x(), p0.y(), p1.y(), p1.z(), ptr));
    sides.add(make_shared<flip_face>(
        make_shared<xy_rect>(p0.x(), p1.x(), p0.y(), p1.y(), p0.z(), ptr)));

    sides.add(make_shared<xz_rect>(p0.x(), p1.x(), p0.z(), p1.z(), p1.y(), ptr));
    sides.add(make_shared<flip_face>(
        make_shared<xz_rect>(p0.x(), p1.x(), p0.z(), p1.z(), p0.y(), ptr)));

    sides.add(make_shared<yz_rect>(p0.y(), p1.y(), p0.z(), p1.z(), p1.x(), ptr));
    sides.add(make_shared<flip_face>(
        make_shared<yz_rect>(p0.y(), p1.y(), p0.z(), p1.z(), p0.x(), ptr)));
}

box::box(const vec3& p0, const vec3& p1, shared_ptr<material> front_ptr, shared_ptr<material> lateral_ptr, shared_ptr<material> top_ptr) {
    box_min = p0;
    box_max = p1;

    sides.add(make_shared<xy_rect>(p0.x(), p1.x(), p0.y(), p1.y(), p1.z(), lateral_ptr));
    sides.add(make_shared<xy_rect>(p0.x(), p1.x(), p0.y(), p1.y(), p0.z(), front_ptr));
    sides.add(make_shared<xz_rect>(p0.x(), p1.x(), p0.z(), p1.z(), p1.y(), top_ptr));
    //sides.add(make_shared<flip_face>(
    //    make_shared<xz_rect>(p0.x(), p1.x(), p0.z(), p1.z(), p0.y(), top_ptr)));

    sides.add(make_shared<yz_rect>(p0.y(), p1.y(), p0.z(), p1.z(), p1.x(), top_ptr));
    sides.add(make_shared<yz_rect>(p0.y(), p1.y(), p0.z(), p1.z(), p0.x(), lateral_ptr));
//    sides.add(make_shared<flip_face>(
//        make_shared<yz_rect>(p0.y(), p1.y(), p0.z(), p1.z(), p0.x(), lateral_ptr)));
}

bool box::hit(const ray& r, double t0, double t1, hit_record& rec) const {
    return sides.hit(r, t0, t1, rec);
}
#endif