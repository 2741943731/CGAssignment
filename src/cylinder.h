#pragma once
#ifndef CYLINDER_H
#define CYLINDER_H

#include "hittable.h"
#include "rtweekend.h"

class cylinder : public hittable {
public:
    cylinder() {}
    cylinder(vec3 cen, vec3 dir, double r, double h, shared_ptr<material> m)
        : center(cen), direction(dir), radius(r), height(h), mat_ptr(m) {
        direction = unit_vector(direction);
    }

    virtual bool hit(const ray& r, double tmin, double tmax, hit_record& rec) const;
    virtual bool bounding_box(double t0, double t1, aabb& output_box) const override;

private:
    vec3 center;
    vec3 direction;
    double radius;
    double height;
    shared_ptr<material> mat_ptr;
};

bool cylinder::hit(const ray& r, double t_min, double t_max, hit_record& rec) const {
    vec3 oc = r.origin() - center;
    auto a = r.direction().length_squared();
    auto half_b = dot(oc, r.direction());
    auto c = oc.length_squared() - radius * radius;

    auto discriminant = half_b * half_b - a * c;
    if (discriminant < 0) return false;
    auto root = sqrt(discriminant);

    // Check both roots
    for (auto temp : { -(-half_b - root) / a, -(-half_b + root) / a }) {
        if (temp < t_max && temp > t_min) {
            rec.t = temp;
            rec.p = r.at(rec.t);

            auto point_on_cylinder = rec.p - center;
            auto projection = dot(point_on_cylinder, direction);
            if (projection < 0 || projection > height) continue;

            vec3 outward_normal = (rec.p - center - projection * direction) / radius;
            rec.set_face_normal(r, outward_normal);
            rec.mat_ptr = mat_ptr;

            // ¼ÆËãuv×ø±ê
            double u = projection / height;
            double phi = atan2(outward_normal.y(), outward_normal.x());
            if (phi < 0) phi += 2 * pi;
            double v = phi / (2 * pi);
            rec.u = u;
            rec.v = v;

            return true;
        }
    }

    return false;
}

bool cylinder::bounding_box(double t0, double t1, aabb& output_box) const {
    vec3 min_point = center - direction * height / 2 - vec3(radius, radius, 0);
    vec3 max_point = center + direction * height / 2 + vec3(radius, radius, 0);
    output_box = aabb(min_point, max_point);
    return true;
}

#endif