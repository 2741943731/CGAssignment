#pragma once
#ifndef OBJ_MESH_H
#define OBJ_MESH_H

#include "hittable.h"
#include "obj_input.h"
#include "vec3.h"
#include "material.h"
#include "triangle.h"
#include "hittable_list.h"
#include "bvh_node.h"

class obj_mesh : public hittable {
public:
    obj_mesh(const std::string& filename, std::shared_ptr<material> mat) : m_material(mat) {
        if (!ReadOBJ::loadOBJ(filename, m_vertices, m_normals, m_texCoords, m_faces)) {
            throw std::runtime_error("Failed to load OBJ file");
        }
        buildBvh();
    }

    virtual bool hit(const ray& r, double t_min, double t_max, hit_record& rec) const override {
        return m_bvh->hit(r, t_min, t_max, rec);
    }

    virtual bool bounding_box(double t0, double t1, aabb& box) const override {
        return m_bvh->bounding_box(t0, t1, box);
    }

private:
    std::shared_ptr<material> m_material;
    std::vector<point3> m_vertices;
    std::vector<vec3> m_normals;
    std::vector<vec3> m_texCoords;
    std::vector<std::vector<int>> m_faces;
    std::shared_ptr<bvh_node> m_bvh;

    void buildBvh() {
        hittable_list objects;
        for (const auto& face : m_faces) {
            if (face.size() != 3) continue; // Skip non-triangular faces for simplicity
            point3 v0 = m_vertices[face[0]];
            point3 v1 = m_vertices[face[1]];
            point3 v2 = m_vertices[face[2]];
            objects.add(make_shared<triangle>(v0, v1, v2, m_material));
        }
        m_bvh = make_shared<bvh_node>(objects, 0, 1);
    }
};

#endif