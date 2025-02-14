#pragma once
//sphere.h
#ifndef SPHERE_H
#define SPHERE_H

#include "hittable.h"
#include "pdf.h"
#include "rtweekend.h"

class sphere : public hittable {
public:
    sphere() {}
    sphere(vec3 cen, double r, shared_ptr<material> m): center(cen), radius(r), mat_ptr(m){};

    virtual bool hit(const ray& r, double tmin, double tmax, hit_record& rec) const;
    virtual bool bounding_box(double t0, double t1, aabb& output_box)const override;

    double sphere::pdf_value(const point3& o, const vec3& v) const {
        hit_record rec;
        if (!this->hit(ray(o, v), 0.001, infinity, rec))
            return 0;

        auto cos_theta_max = sqrt(1 - radius * radius / (center - o).length_squared());
        auto solid_angle = 2 * pi * (1 - cos_theta_max);

        return  1 / solid_angle;
    }

    vec3 sphere::random(const point3& o) const {
        vec3 direction = center - o;
        auto distance_squared = direction.length_squared();
        onb uvw;
        uvw.build_from_w(direction);
        return uvw.local(random_to_sphere(radius, distance_squared));
    }
public:
    vec3 center;
    double radius;
    shared_ptr<material> mat_ptr;
};

bool sphere::hit(const ray& r, double t_min, double t_max, hit_record& rec) const {
    vec3 oc = r.origin() - center;
    auto a = r.direction().length_squared();
    auto half_b = dot(oc, r.direction());
    auto c = oc.length_squared() - radius * radius;
    auto discriminant = half_b * half_b - a * c;
    //get_sphere_uv((rec.p - center) / radius, rec.u, rec.v);

    if (discriminant > 0) {
        auto root = sqrt(discriminant);
        auto temp = (-half_b - root) / a;
        if (temp < t_max && temp > t_min) {
            rec.t = temp;
            rec.p = r.at(rec.t);
            vec3 outward_normal = (rec.p - center) / radius;
            rec.set_face_normal(r, outward_normal);
            rec.mat_ptr = mat_ptr;
            get_sphere_uv((rec.p - center) / radius, rec.u, rec.v);
            return true;
        }
        temp = (-half_b + root) / a;
        if (temp < t_max && temp > t_min) {
            rec.t = temp;
            rec.p = r.at(rec.t);
            vec3 outward_normal = (rec.p - center) / radius;
            rec.set_face_normal(r, outward_normal);
            rec.mat_ptr = mat_ptr;
            get_sphere_uv((rec.p - center) / radius, rec.u, rec.v);
            return true;
        }
    }
    return false;
}

bool sphere::bounding_box(double t0, double t1, aabb& output_box) const {
    output_box = aabb(
        center - vec3(radius, radius, radius),
        center + vec3(radius, radius, radius));
    return true;
}

#endif