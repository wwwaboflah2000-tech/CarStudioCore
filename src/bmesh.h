#ifndef BMESH_H
#define BMESH_H

#include <godot_cpp/variant/vector3.hpp>
#include <vector>

namespace godot {

using VertId = int;
using EdgeId = int;
using FaceId = int;
using LoopId = int;

struct BMVert {
    int id;
    Vector3 co;
    EdgeId edge = -1;
    bool deleted = false;
};

struct BMEdge {
    int id;
    VertId v1 = -1;
    VertId v2 = -1;
    LoopId l = -1;
    bool deleted = false;
};

struct BMLoop {
    int id;
    VertId v = -1;
    EdgeId e = -1;
    FaceId f = -1;
    LoopId radial_next = -1;
    LoopId radial_prev = -1;
    LoopId next = -1;
    LoopId prev = -1;
};

struct BMFace {
    int id;
    LoopId l_first = -1;
    int len = 0;
    Vector3 normal = Vector3(0, 1, 0);
    bool deleted = false;
};

class BMesh {
public:
    std::vector<BMVert> verts;
    std::vector<BMEdge> edges;
    std::vector<BMLoop> loops;
    std::vector<BMFace> faces;

    BMesh() = default;

    VertId add_vert(const Vector3& co);
    EdgeId add_edge(VertId v1, VertId v2);
    FaceId add_face(const std::vector<VertId>& vert_ids);
    
    Vector3 calc_face_normal(const BMFace& f) const;
    void create_cube(float size);

    FaceId make_smart_face(const std::vector<VertId>& selected_verts);
    FaceId extrude_face(FaceId face_id, float distance);
    FaceId extrude_edge(EdgeId edge_id, float distance);
    EdgeId extrude_vert(VertId vert_id, const Vector3& offset);
};

} // namespace godot

#endif
