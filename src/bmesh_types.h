#ifndef BMESH_TYPES_H
#define BMESH_TYPES_H

#include <godot_cpp/variant/vector3.hpp>

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

} // namespace godot

#endif
