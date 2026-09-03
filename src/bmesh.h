#ifndef BMESH_H
#define BMESH_H

#include "bmesh_types.h"
#include <vector>

namespace godot {

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
