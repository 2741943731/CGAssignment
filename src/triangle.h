#pragma once
#ifndef TRIANGLE_H
#define TRIANGLE_H

#include "hittable.h"
#include "aabb.h"
#include "rtweekend.h"
#include "obj_input.h"

vec3 face_normal(const vec3& v1, const vec3& v2, const vec3& v3) {
    vec3 a = v2 - v1;
    vec3 b = v3 - v1;
    vec3 n = cross(a, b);
    return unit_vector(n);
}

class triangle : public hittable {
public:
    triangle(const vec3& v0, const vec3& v1, const vec3& v2, shared_ptr<material> mat)
        : vertex0(v0), vertex1(v1), vertex2(v2), mat_ptr(mat) {}

    virtual bool hit(const ray& r, double t_min, double t_max, hit_record& rec) const override {
        vec3 edge1 = vertex1 - vertex0;
        vec3 edge2 = vertex2 - vertex0;
        vec3 h = cross(r.direction(), edge2);
        double a = dot(edge1, h);
        if (a > -0.00001 && a < 0.00001)
            return false;
        double f = 1.0 / a;
        vec3 s = r.origin() - vertex0;
        double u = f * dot(s, h);
        if (u < 0.0 || u > 1.0)
            return false;
        vec3 q = cross(s, edge1);
        double v = f * dot(r.direction(), q);
        if (v < 0.0 || u + v > 1.0)
            return false;
        double t = f * dot(edge2, q);
        if (t < t_min || t > t_max)
            return false;
        rec.t = t;
        rec.p = r.at(t);
        rec.mat_ptr = mat_ptr;
        vec3 normal = face_normal(vertex0, vertex1, vertex2);
        rec.set_face_normal(r, normal);
        return true;
    }

    virtual bool bounding_box(double t0, double t1, aabb& box) const override {
        vec3 min = minxyz(minxyz(vertex0, vertex1), vertex2);
        vec3 max = maxxyz(maxxyz(vertex0, vertex1), vertex2);
        box = aabb(min, max);
        return true;
    }

private:
    vec3 vertex0;
    vec3 vertex1;
    vec3 vertex2;
    shared_ptr<material> mat_ptr;
};

void rotation(point3 &vertice, std::vector<std::vector<double>> matrix) {
    point3 temp;
    double x = vertice.x(), y = vertice.y(), z = vertice.z();
    std::cout << x << " " << y << " " << z << std::endl;
    temp.e[0] = x * matrix[0][0] + y * matrix[0][1] + z * matrix[0][2];
    temp.e[1] = x * matrix[1][0] + y * matrix[1][1] + z * matrix[1][2];
    temp.e[2] = x * matrix[2][0] + y * matrix[2][1] + z * matrix[2][2];
    vertice = temp;
}

static hittable_list load_obj_model(const std::string& filename, std::shared_ptr<material> mat, hittable_list& world, std::vector<std::vector<double>> matrix, vec3 move) {
    std::vector<point3> vertices;
    std::vector<vec3> normals;
    std::vector<vec3> texCoords;
    std::vector<std::vector<int>> faces;
    std::map<std::vector<int>, vec3> materialFaces;

    OBJINPUT::loadOBJ(filename, vertices, normals, texCoords, faces, materialFaces);

    //std::cout << "test" << std::endl;

    for (int i = 0; i < vertices.size(); ++i) {
        rotation(vertices[i], matrix);
        vertices[i] += move;
    }

    //hittable_list obj_triangles;
    for (const auto& face : faces) {
        for (size_t i = 0; i < face.size() - 2; ++i) {
            int v0_idx = face[0], v1_idx = face[i + 1], v2_idx = face[i + 2];
            //rotation(vertices[v0_idx], matrix);
            //rotation(vertices[v1_idx], matrix);
            //rotation(vertices[v2_idx], matrix);
            vec3 tempColor = materialFaces[face];
            world.add(make_shared<triangle>(
                vertices[v0_idx], vertices[v1_idx], vertices[v2_idx], make_shared<lambertian>(tempColor)));
        }
    }

    return world;
}

#endif