#include "bmesh.h"
#include <cmath>
#include <algorithm>

namespace godot {

VertId BMesh::add_vert(const Vector3& co) {
    int id = static_cast<int>(verts.size());
    verts.push_back({id, co, -1, false});
    return id;
}

EdgeId BMesh::add_edge(VertId v1, VertId v2) {
    for (const auto& e : edges) {
        if (!e.deleted && ((e.v1 == v1 && e.v2 == v2) || (e.v1 == v2 && e.v2 == v1))) {
            return e.id;
        }
    }
    int id = static_cast<int>(edges.size());
    edges.push_back({id, v1, v2, -1, false});
    verts[v1].edge = id;
    verts[v2].edge = id;
    return id;
}

FaceId BMesh::add_face(const std::vector<VertId>& vert_ids) {
    int n = static_cast<int>(vert_ids.size());
    if (n < 3) return -1;

    int face_id = static_cast<int>(faces.size());
    std::vector<EdgeId> edge_ids(n);
    for (int i = 0; i < n; ++i) {
        edge_ids[i] = add_edge(vert_ids[i], vert_ids[(i + 1) % n]);
    }

    int start_loop = static_cast<int>(loops.size());
    for (int i = 0; i < n; ++i) {
        LoopId l_id = start_loop + i;
        LoopId next_l = start_loop + (i + 1) % n;
        LoopId prev_l = start_loop + (i + n - 1) % n;
        loops.push_back({l_id, vert_ids[i], edge_ids[i], face_id, l_id, l_id, next_l, prev_l});
    }

    for (int i = 0; i < n; ++i) {
        LoopId cur_l = start_loop + i;
        EdgeId e_idx = edge_ids[i];
        if (edges[e_idx].l != -1) {
            LoopId first_l = edges[e_idx].l;
            LoopId prev_l = loops[first_l].radial_prev;
            loops[cur_l].radial_next = first_l;
            loops[cur_l].radial_prev = prev_l;
            loops[prev_l].radial_next = cur_l;
            loops[first_l].radial_prev = cur_l;
        } else {
            edges[e_idx].l = cur_l;
        }
    }

    BMFace f;
    f.id = face_id;
    f.l_first = start_loop;
    f.len = n;
    f.deleted = false;
    f.normal = calc_face_normal(f);
    faces.push_back(f);
    return face_id;
}

Vector3 BMesh::calc_face_normal(const BMFace& f) const {
    Vector3 norm(0, 0, 0);
    LoopId cur = f.l_first;
    for (int i = 0; i < f.len; ++i) {
        const BMLoop& l = loops[cur];
        const BMLoop& nl = loops[l.next];
        Vector3 v0 = verts[l.v].co;
        Vector3 v1 = verts[nl.v].co;
        norm += (v0 - v1).cross(v0 + v1);
        cur = l.next;
    }
    return (norm.length_squared() > 1e-6f) ? norm.normalized() : Vector3(0, 1, 0);
}

FaceId BMesh::make_smart_face(const std::vector<VertId>& selected_verts) {
    if (selected_verts.size() < 3) {
        if (selected_verts.size() == 2) {
            add_edge(selected_verts[0], selected_verts[1]);
        }
        return -1;
    }

    Vector3 centroid(0, 0, 0);
    for (VertId v : selected_verts) {
        centroid += verts[v].co;
    }
    centroid /= static_cast<float>(selected_verts.size());

    Vector3 normal(0, 0, 0);
    int n = static_cast<int>(selected_verts.size());
    for (int i = 0; i < n; ++i) {
        Vector3 c = verts[selected_verts[i]].co;
        Vector3 nxt = verts[selected_verts[(i + 1) % n]].co;
        normal.x += (c.y - nxt.y) * (c.z + nxt.z);
        normal.y += (c.z - nxt.z) * (c.x + nxt.x);
        normal.z += (c.x - nxt.x) * (c.y + nxt.y);
    }
    if (normal.length_squared() < 1e-6f) {
        normal = Vector3(0, 1, 0);
    } else {
        normal = normal.normalized();
    }

    Vector3 u_axis = (std::abs(normal.y) < 0.9f) ? Vector3(0, 1, 0).cross(normal).normalized() : Vector3(1, 0, 0).cross(normal).normalized();
    Vector3 v_axis = normal.cross(u_axis).normalized();

    struct AngleVert {
        VertId id;
        float angle;
    };
    std::vector<AngleVert> sorted;
    for (VertId v : selected_verts) {
        Vector3 d = verts[v].co - centroid;
        float x = d.dot(u_axis);
        float y = d.dot(v_axis);
        sorted.push_back({v, std::atan2(y, x)});
    }

    std::sort(sorted.begin(), sorted.end(), [](const AngleVert& a, const AngleVert& b) {
        return a.angle < b.angle;
    });

    std::vector<VertId> clean_verts;
    for (const auto& av : sorted) {
        clean_verts.push_back(av.id);
    }

    return add_face(clean_verts);
}

FaceId BMesh::extrude_face(FaceId face_id, float distance) {
    if (face_id < 0 || face_id >= static_cast<int>(faces.size()) || faces[face_id].deleted) {
        return -1;
    }
    BMFace& old_f = faces[face_id];
    Vector3 offset = old_f.normal * distance;

    std::vector<VertId> base_verts;
    LoopId cur = old_f.l_first;
    for (int i = 0; i < old_f.len; ++i) {
        base_verts.push_back(loops[cur].v);
        cur = loops[cur].next;
    }

    std::vector<VertId> top_verts;
    for (VertId bv : base_verts) {
        top_verts.push_back(add_vert(verts[bv].co + offset));
    }

    old_f.deleted = true;
    int n = static_cast<int>(base_verts.size());
    for (int i = 0; i < n; ++i) {
        add_face({base_verts[i], base_verts[(i + 1) % n], top_verts[(i + 1) % n], top_verts[i]});
    }

    return add_face(top_verts);
}

FaceId BMesh::extrude_edge(EdgeId edge_id, float distance) {
    if (edge_id < 0 || edge_id >= static_cast<int>(edges.size()) || edges[edge_id].deleted) {
        return -1;
    }
    BMEdge& e = edges[edge_id];
    Vector3 edge_dir = (verts[e.v2].co - verts[e.v1].co).normalized();
    Vector3 norm = Vector3(0, 1, 0).cross(edge_dir).normalized();
    if (norm.length_squared() < 0.01f) {
        norm = Vector3(1, 0, 0);
    }

    VertId v3 = add_vert(verts[e.v2].co + norm * distance);
    VertId v4 = add_vert(verts[e.v1].co + norm * distance);

    return add_face({e.v1, e.v2, v3, v4});
}

EdgeId BMesh::extrude_vert(VertId vert_id, const Vector3& offset) {
    if (vert_id < 0 || vert_id >= static_cast<int>(verts.size()) || verts[vert_id].deleted) {
        return -1;
    }
    VertId new_v = add_vert(verts[vert_id].co + offset);
    return add_edge(vert_id, new_v);
}

void BMesh::create_cube(float size) {
    verts.clear();
    edges.clear();
    loops.clear();
    faces.clear();

    float h = size * 0.5f;
    VertId v[8] = {
        add_vert(Vector3(-h, 0,  h)), add_vert(Vector3( h, 0,  h)),
        add_vert(Vector3( h, 0, -h)), add_vert(Vector3(-h, 0, -h)),
        add_vert(Vector3(-h, size,  h)), add_vert(Vector3( h, size,  h)),
        add_vert(Vector3( h, size, -h)), add_vert(Vector3(-h, size, -h))
    };
    add_face({v[0], v[1], v[5], v[4]});
    add_face({v[1], v[2], v[6], v[5]});
    add_face({v[2], v[3], v[7], v[6]});
    add_face({v[3], v[0], v[4], v[7]});
    add_face({v[4], v[5], v[6], v[7]});
    add_face({v[3], v[2], v[1], v[0]});
}

} // namespace godot
