#include "car_modeler.h"
#include <godot_cpp/core/class_db.hpp>
#include <godot_cpp/classes/surface_tool.hpp>
#include <godot_cpp/classes/cylinder_mesh.hpp>
#include <godot_cpp/classes/standard_material3d.hpp>
#include <godot_cpp/classes/input_event_screen_touch.hpp>
#include <godot_cpp/classes/input_event_screen_drag.hpp>
#include <algorithm>
#include <cmath>

namespace godot {

// خوارزمية Möller–Trumbore الدقيقة لفحص تقاطع شعاع الكاميرا مع الأوجه
static bool ray_triangle_intersect(const Vector3& orig, const Vector3& dir,
                                   const Vector3& v0, const Vector3& v1, const Vector3& v2,
                                   float& t) {
    Vector3 e1 = v1 - v0;
    Vector3 e2 = v2 - v0;
    Vector3 pvec = dir.cross(e2);
    float det = e1.dot(pvec);
    if (std::abs(det) < 1e-7f) return false;
    float inv_det = 1.0f / det;
    Vector3 tvec = orig - v0;
    float u = tvec.dot(pvec) * inv_det;
    if (u < 0.0f || u > 1.0f) return false;
    Vector3 qvec = tvec.cross(e1);
    float v = dir.dot(qvec) * inv_det;
    if (v < 0.0f || u + v > 1.0f) return false;
    t = e2.dot(qvec) * inv_det;
    return (t > 1e-3f);
}

// مسافة فراغية بين شعاع وقطعة مستقيمة لالتقاط محاور الجزمو بدقة
static float dist_ray_to_segment(const Vector3& ro, const Vector3& rd, const Vector3& p0, const Vector3& p1) {
    Vector3 u = rd;
    Vector3 v = p1 - p0;
    Vector3 w = ro - p0;
    float a = u.dot(u), b = u.dot(v), c = v.dot(v), d = u.dot(w), e = v.dot(w);
    float D = a * c - b * b;
    float sc, tc;
    if (D < 1e-6f) {
        sc = 0.0f;
        tc = (b > c ? d / b : e / c);
    } else {
        sc = (b * e - c * d) / D;
        tc = (a * e - b * d) / D;
    }
    if (sc < 0.0f) sc = 0.0f;
    tc = std::clamp(tc, 0.0f, 1.0f);
    return (w + (u * sc) - (v * tc)).length();
}

void CarModeler::_bind_methods() {
    ClassDB::bind_method(D_METHOD("_on_btn_vertex_mode_pressed"), &CarModeler::_on_btn_vertex_mode_pressed);
    ClassDB::bind_method(D_METHOD("_on_btn_edge_mode_pressed"), &CarModeler::_on_btn_edge_mode_pressed);
    ClassDB::bind_method(D_METHOD("_on_btn_face_mode_pressed"), &CarModeler::_on_btn_face_mode_pressed);
    ClassDB::bind_method(D_METHOD("_on_btn_object_mode_pressed"), &CarModeler::_on_btn_object_mode_pressed);
    ClassDB::bind_method(D_METHOD("_on_btn_ctrl_toggled", "toggled_on"), &CarModeler::_on_btn_ctrl_toggled);
    ClassDB::bind_method(D_METHOD("_on_btn_make_face_pressed"), &CarModeler::_on_btn_make_face_pressed);
    ClassDB::bind_method(D_METHOD("_on_btn_extrude_pressed"), &CarModeler::_on_btn_extrude_pressed);
    ClassDB::bind_method(D_METHOD("_on_btn_delete_pressed"), &CarModeler::_on_btn_delete_pressed);
}

CarModeler::CarModeler() {}
CarModeler::~CarModeler() {}

void CarModeler::_ready() {
    m_camera = get_node<Camera3D>("Camera3D");
    m_car_mesh = get_node<MeshInstance3D>("CarMesh");
    m_gizmo_root = get_node<Node3D>("GizmoRoot");
    m_lbl_status = get_node<Label>("UI/BottomToast/LblStatus");
    m_btn_ctrl = get_node<Button>("UI/BtnCtrl");

    setup_gizmo_nodes();
    update_camera_transform();

    m_bmesh.create_cube(1.5f);
    m_selected_faces.insert(4); // تحديد الوجه العلوي افتراضياً
    
    rebuild_render_mesh();
    update_gizmo();

    if (m_lbl_status) {
        m_lbl_status->set_text("🟢 BMesh جاهز: اسحب المحاور لتحريك الوجه");
    }
}

// بناء محاور الجزمو الثلاثة (أحمر، أخضر، أزرق)
void CarModeler::setup_gizmo_nodes() {
    if (!m_gizmo_root) return;

    for (int i = 0; i < m_gizmo_root->get_child_count(); ++i) {
        m_gizmo_root->get_child(i)->queue_free();
    }

    auto make_axis = [this](const Vector3& dir, const Color& col, const Vector3& rot_deg) {
        Ref<CylinderMesh> shaft;
        shaft.instantiate();
        shaft->set_top_radius(0.02f);
        shaft->set_bottom_radius(0.02f);
        shaft->set_height(0.6f);

        Ref<StandardMaterial3D> mat;
        mat.instantiate();
        mat->set_shading_mode(BaseMaterial3D::SHADING_MODE_UNSHADED);
        mat->set_albedo(col);
        mat->set_flag(BaseMaterial3D::FLAG_DISABLE_DEPTH_TEST, true); // يظهر دائماً في المقدمة كبلندر
        mat->set_render_priority(20);

        MeshInstance3D* mi = memnew(MeshInstance3D);
        mi->set_mesh(shaft);
        mi->set_material_override(mat);
        mi->set_position(dir * 0.3f);
        mi->set_rotation_degrees(rot_deg);
        m_gizmo_root->add_child(mi);

        // رأس السهم المخروطي
        Ref<CylinderMesh> cone;
        cone.instantiate();
        cone->set_top_radius(0.0f);
        cone->set_bottom_radius(0.06f);
        cone->set_height(0.18f);

        MeshInstance3D* mi_c = memnew(MeshInstance3D);
        mi_c->set_mesh(cone);
        mi_c->set_material_override(mat);
        mi_c->set_position(dir * 0.65f);
        mi_c->set_rotation_degrees(rot_deg);
        m_gizmo_root->add_child(mi_c);
    };

    make_axis(Vector3(1, 0, 0), Color(0.95f, 0.2f, 0.2f), Vector3(0, 0, -90)); // X أحمر
    make_axis(Vector3(0, 1, 0), Color(0.2f, 0.9f, 0.2f), Vector3(0, 0, 0));    // Y أخضر
    make_axis(Vector3(0, 0, 1), Color(0.2f, 0.4f, 0.95f), Vector3(90, 0, 0));   // Z أزرق
}

void CarModeler::update_camera_transform() {
    if (!m_camera) return;
    Vector3 target(0, 0.75f, 0);
    Basis rot = Basis::from_euler(Vector3(m_cam_pitch, m_cam_yaw, 0));
    m_camera->set_global_position(target + rot.xform(Vector3(0, 0, m_cam_dist)));
    m_camera->look_at(target, Vector3(0, 1, 0));
    update_gizmo();
}

void CarModeler::update_gizmo() {
    if (!m_gizmo_root) return;

    if (m_selected_faces.empty() && m_selected_verts.empty() && m_selected_edges.empty()) {
        m_gizmo_root->set_visible(false);
        return;
    }

    // حساب مركز ثقل التحديد (Median Pivot)
    Vector3 center(0, 0, 0);
    int count = 0;
    std::set<VertId> unique_verts;

    for (FaceId fid : m_selected_faces) {
        if (fid < 0 || fid >= (int)m_bmesh.faces.size() || m_bmesh.faces[fid].deleted) continue;
        LoopId cur = m_bmesh.faces[fid].l_first;
        for (int i = 0; i < m_bmesh.faces[fid].len; ++i) {
            unique_verts.insert(m_bmesh.loops[cur].v);
            cur = m_bmesh.loops[cur].next;
        }
    }

    for (VertId vid : unique_verts) {
        center += m_bmesh.verts[vid].co;
        count++;
    }

    if (count > 0) {
        m_gizmo_pos = center / (float)count;
        m_gizmo_root->set_global_position(m_gizmo_pos);

        // مقياس ديناميكي متكيف مع بعد الكاميرا
        float s = std::clamp(m_cam_dist * 0.16f, 0.3f, 2.5f);
        m_gizmo_root->set_scale(Vector3(s, s, s));
        m_gizmo_root->set_visible(true);
    } else {
        m_gizmo_root->set_visible(false);
    }
}

Vector3 CarModeler::get_ray_plane_intersection(const Vector3& ray_origin, const Vector3& ray_dir, const Vector3& plane_point, const Vector3& plane_normal) {
    float denom = plane_normal.dot(ray_dir);
    if (std::abs(denom) > 1e-6f) {
        float t = (plane_point - ray_origin).dot(plane_normal) / denom;
        return ray_origin + ray_dir * t;
    }
    return plane_point;
}

int CarModeler::pick_gizmo_axis(const Vector2& screen_pos) {
    if (!m_camera || !m_gizmo_root || !m_gizmo_root->is_visible()) return -1;

    Vector3 ray_from = m_camera->project_ray_origin(screen_pos);
    Vector3 ray_dir = m_camera->project_ray_normal(screen_pos).normalized();

    float s = m_gizmo_root->get_scale().x;
    Vector3 axes[3] = { Vector3(1, 0, 0), Vector3(0, 1, 0), Vector3(0, 0, 1) };
    
    int best_axis = -1;
    float min_dist = 0.18f * s; // منطقة سماح لمس مريحة للشاشة

    for (int i = 0; i < 3; ++i) {
        Vector3 p0 = m_gizmo_pos;
        Vector3 p1 = m_gizmo_pos + axes[i] * (0.8f * s);
        float d = dist_ray_to_segment(ray_from, ray_dir, p0, p1);
        if (d < min_dist) {
            min_dist = d;
            best_axis = i;
        }
    }
    return best_axis;
}

FaceId CarModeler::pick_face_at_screen_pos(const Vector2& screen_pos) {
    if (!m_camera) return -1;

    Vector3 ray_from = m_camera->project_ray_origin(screen_pos);
    Vector3 ray_dir = m_camera->project_ray_normal(screen_pos).normalized();

    FaceId best_face = -1;
    float min_t = 1e9f;

    for (const auto& f : m_bmesh.faces) {
        if (f.deleted || f.len < 3) continue;

        std::vector<Vector3> pts;
        LoopId cur = f.l_first;
        for (int i = 0; i < f.len; ++i) {
            pts.push_back(m_bmesh.verts[m_bmesh.loops[cur].v].co);
            cur = m_bmesh.loops[cur].next;
        }

        for (size_t j = 1; j < pts.size() - 1; ++j) {
            float t = 0.0f;
            if (ray_triangle_intersect(ray_from, ray_dir, pts[0], pts[j], pts[j + 1], t)) {
                if (t < min_t) {
                    min_t = t;
                    best_face = f.id;
                }
            }
        }
    }
    return best_face;
}

void CarModeler::_unhandled_input(const Ref<InputEvent>& event) {
    Ref<InputEventScreenTouch> touch = event;
    if (touch.is_valid()) {
        if (touch->is_pressed()) {
            m_is_touching = true;
            m_touch_start_pos = touch->get_position();
            m_total_drag_dist = 0.0f;

            // فحص إذا لمس المستخدم أحد محاور الجزمو
            int axis = pick_gizmo_axis(touch->get_position());
            if (axis != -1) {
                m_active_gizmo_axis = axis;
                m_is_dragging_gizmo = true;
                Vector3 cam_fwd = -m_camera->get_global_transform().basis.get_column(2).normalized();
                Vector3 ray_from = m_camera->project_ray_origin(touch->get_position());
                Vector3 ray_dir = m_camera->project_ray_normal(touch->get_position()).normalized();
                m_gizmo_prev_hit = get_ray_plane_intersection(ray_from, ray_dir, m_gizmo_pos, cam_fwd);
            }
        } else {
            m_is_touching = false;
            m_is_dragging_gizmo = false;
            m_active_gizmo_axis = -1;

            // نقرة سريعة على مجسم (تحديد أو إلغاء)
            if (m_total_drag_dist < 12.0f) {
                FaceId hit_face = pick_face_at_screen_pos(touch->get_position());
                if (hit_face != -1) {
                    if (m_ctrl_active) {
                        if (m_selected_faces.count(hit_face)) m_selected_faces.erase(hit_face);
                        else m_selected_faces.insert(hit_face);
                    } else {
                        m_selected_faces.clear();
                        m_selected_faces.insert(hit_face);
                    }
                    rebuild_render_mesh();
                    update_gizmo();
                    if (m_lbl_status) m_lbl_status->set_text("🎯 تم تحديد الوجه: " + String::num_int64(hit_face));
                } else if (!m_ctrl_active) {
                    m_selected_faces.clear();
                    rebuild_render_mesh();
                    update_gizmo();
                    if (m_lbl_status) m_lbl_status->set_text("تم إلغاء التحديد");
                }
            }
        }
        return;
    }

    Ref<InputEventScreenDrag> drag = event;
    if (drag.is_valid() && m_is_touching) {
        m_total_drag_dist += drag->get_relative().length();

        // 1. تحريك الجزمو وسحب الوجه في الفضاء
        if (m_is_dragging_gizmo && m_active_gizmo_axis != -1) {
            Vector3 cam_fwd = -m_camera->get_global_transform().basis.get_column(2).normalized();
            Vector3 ray_from = m_camera->project_ray_origin(drag->get_position());
            Vector3 ray_dir = m_camera->project_ray_normal(drag->get_position()).normalized();
            Vector3 hit = get_ray_plane_intersection(ray_from, ray_dir, m_gizmo_pos, cam_fwd);

            Vector3 world_delta = hit - m_gizmo_prev_hit;
            Vector3 axis_dir = (m_active_gizmo_axis == 0 ? Vector3(1,0,0) : (m_active_gizmo_axis == 1 ? Vector3(0,1,0) : Vector3(0,0,1)));
            Vector3 move_vec = axis_dir * world_delta.dot(axis_dir);

            // تحريك كل رؤوس الأوجه المحددة
            std::set<VertId> unique_verts;
            for (FaceId fid : m_selected_faces) {
                if (fid < 0 || fid >= (int)m_bmesh.faces.size() || m_bmesh.faces[fid].deleted) continue;
                LoopId cur = m_bmesh.faces[fid].l_first;
                for (int i = 0; i < m_bmesh.faces[fid].len; ++i) {
                    unique_verts.insert(m_bmesh.loops[cur].v);
                    cur = m_bmesh.loops[cur].next;
                }
            }

            for (VertId vid : unique_verts) {
                m_bmesh.verts[vid].co += move_vec;
            }

            m_gizmo_prev_hit = hit;
            rebuild_render_mesh();
            update_gizmo();
            return;
        }

        // 2. تدوير الكاميرا حول المجسم إذا لم نكن نسحب الجزمو
        Vector2 rel = drag->get_relative();
        m_cam_yaw -= rel.x * 0.005f;
        m_cam_pitch = std::clamp(m_cam_pitch - rel.y * 0.005f, -1.4f, 1.4f);
        update_camera_transform();
        return;
    }
}

void CarModeler::_on_btn_ctrl_toggled(bool toggled_on) {
    m_ctrl_active = toggled_on;
    if (m_btn_ctrl) {
        m_btn_ctrl->set_text(m_ctrl_active ? "Ctrl: ON" : "Ctrl: OFF");
    }
}

void CarModeler::_on_btn_vertex_mode_pressed() { m_mode = 0; m_selected_verts.clear(); rebuild_render_mesh(); update_gizmo(); }
void CarModeler::_on_btn_edge_mode_pressed()   { m_mode = 1; m_selected_edges.clear(); rebuild_render_mesh(); update_gizmo(); }
void CarModeler::_on_btn_face_mode_pressed()   { m_mode = 2; m_selected_faces.clear(); rebuild_render_mesh(); update_gizmo(); }
void CarModeler::_on_btn_object_mode_pressed() { m_mode = 3; rebuild_render_mesh(); update_gizmo(); }

void CarModeler::_on_btn_make_face_pressed() {
    if (m_mode == 0 && m_selected_verts.size() >= 2) {
        std::vector<VertId> v_list(m_selected_verts.begin(), m_selected_verts.end());
        FaceId fid = m_bmesh.make_smart_face(v_list);
        if (fid != -1) {
            m_selected_verts.clear();
            m_selected_faces.insert(fid);
            m_mode = 2;
            rebuild_render_mesh();
            update_gizmo();
            if (m_lbl_status) m_lbl_status->set_text("✨ تم إنشاء الوجه بنجاح!");
        }
    }
}

void CarModeler::_on_btn_extrude_pressed() {
    if (m_mode == 2 && !m_selected_faces.empty()) {
        FaceId f = *m_selected_faces.begin();
        FaceId new_f = m_bmesh.extrude_face(f, 0.4f);
        if (new_f != -1) {
            m_selected_faces.clear();
            m_selected_faces.insert(new_f);
            rebuild_render_mesh();
            update_gizmo();
            if (m_lbl_status) m_lbl_status->set_text("⬆️ تم بثق الوجه");
        }
    }
}

void CarModeler::_on_btn_delete_pressed() {
    for (FaceId f : m_selected_faces) {
        m_bmesh.faces[f].deleted = true;
    }
    m_selected_faces.clear();
    rebuild_render_mesh();
    update_gizmo();
}

void CarModeler::rebuild_render_mesh() {
    if (!m_car_mesh) return;

    Ref<SurfaceTool> st;
    st.instantiate();
    st->begin(Mesh::PRIMITIVE_TRIANGLES);

    Color default_col(0.72f, 0.76f, 0.82f);
    Color select_col(1.0f, 0.55f, 0.15f);

    for (const auto& f : m_bmesh.faces) {
        if (f.deleted || f.len < 3) continue;

        Color col = (m_mode == 2 && m_selected_faces.count(f.id)) ? select_col : default_col;

        std::vector<Vector3> pts;
        LoopId cur = f.l_first;
        for (int i = 0; i < f.len; ++i) {
            pts.push_back(m_bmesh.verts[m_bmesh.loops[cur].v].co);
            cur = m_bmesh.loops[cur].next;
        }

        if (f.len == 4) {
            // رسم رباعي نظيف دون خطوط وترية وهمية
            Vector3 v0 = pts[0], v1 = pts[1], v2 = pts[2], v3 = pts[3];

            st->set_normal(f.normal); st->set_color(col); st->set_uv(Vector2(0, 0)); st->add_vertex(v0);
            st->set_normal(f.normal); st->set_color(col); st->set_uv(Vector2(1, 0)); st->add_vertex(v1);
            st->set_normal(f.normal); st->set_color(col); st->set_uv(Vector2(1, 1)); st->add_vertex(v2);

            st->set_normal(f.normal); st->set_color(col); st->set_uv(Vector2(0, 0)); st->add_vertex(v0);
            st->set_normal(f.normal); st->set_color(col); st->set_uv(Vector2(1, 1)); st->add_vertex(v2);
            st->set_normal(f.normal); st->set_color(col); st->set_uv(Vector2(0, 1)); st->add_vertex(v3);
        } else {
            for (size_t j = 1; j < pts.size() - 1; ++j) {
                st->set_normal(f.normal); st->set_color(col); st->set_uv(Vector2(0, 0)); st->add_vertex(pts[0]);
                st->set_normal(f.normal); st->set_color(col); st->set_uv(Vector2(1, 0)); st->add_vertex(pts[j]);
                st->set_normal(f.normal); st->set_color(col); st->set_uv(Vector2(0.5, 1)); st->add_vertex(pts[j + 1]);
            }
        }
    }

    m_car_mesh->set_mesh(st->commit());
}

} // namespace godot
