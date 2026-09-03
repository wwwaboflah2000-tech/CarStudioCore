#include "car_modeler.h"
#include <godot_cpp/core/class_db.hpp>
#include <godot_cpp/classes/surface_tool.hpp>
#include <godot_cpp/classes/input_event_screen_touch.hpp>
#include <godot_cpp/classes/input_event_screen_drag.hpp>
#include <godot_cpp/classes/input_event_mouse_button.hpp>
#include <godot_cpp/classes/input_event_mouse_motion.hpp>
#include <algorithm>
#include <cmath>

namespace godot {

// خوارزمية Möller–Trumbore الدقيقة لفحص تقاطع شعاع الكاميرا مع المضلع
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
    m_lbl_status = get_node<Label>("UI/BottomToast/LblStatus");
    m_btn_ctrl = get_node<Button>("UI/BtnCtrl");

    update_camera_transform();
    m_bmesh.create_cube(1.5f);
    m_selected_faces.insert(4); // تحديد الوجه العلوي افتراضياً
    rebuild_render_mesh();

    if (m_lbl_status) {
        m_lbl_status->set_text("🟢 المس أي وجه لتحديده أو اضغط بثق");
    }
}

void CarModeler::update_camera_transform() {
    if (!m_camera) return;
    Vector3 target(0, 0.75f, 0);
    Basis rot = Basis::from_euler(Vector3(m_cam_pitch, m_cam_yaw, 0));
    m_camera->set_global_position(target + rot.xform(Vector3(0, 0, m_cam_dist)));
    m_camera->look_at(target, Vector3(0, 1, 0));
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

        // فحص كل مثلثات الوجه
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
    // 1. دعم اللمس على شاشات الهاتف
    Ref<InputEventScreenTouch> touch = event;
    if (touch.is_valid()) {
        if (touch->is_pressed()) {
            m_is_touching = true;
            m_touch_start_pos = touch->get_position();
            m_total_drag_dist = 0.0f;
        } else {
            m_is_touching = false;
            // إذا كانت المسافة المقطوعة صغيرة جداً = هذه نقرة وليست سحباً للكاميرا
            if (m_total_drag_dist < 12.0f) {
                FaceId hit_face = pick_face_at_screen_pos(touch->get_position());
                if (hit_face != -1) {
                    if (m_ctrl_active) {
                        // وضع التحديد المتعدد: إضافة أو إزالة
                        if (m_selected_faces.count(hit_face)) m_selected_faces.erase(hit_face);
                        else m_selected_faces.insert(hit_face);
                    } else {
                        m_selected_faces.clear();
                        m_selected_faces.insert(hit_face);
                    }
                    rebuild_render_mesh();
                    if (m_lbl_status) m_lbl_status->set_text("🎯 تم تحديد الوجه رقم: " + String::num_int64(hit_face));
                } else if (!m_ctrl_active) {
                    // نقر في الفراغ يلغي التحديد إذا كان Ctrl غير مفعّل
                    m_selected_faces.clear();
                    rebuild_render_mesh();
                    if (m_lbl_status) m_lbl_status->set_text("تم إلغاء التحديد");
                }
            }
        }
        return;
    }

    // 2. سحب الإصبع لتدوير الكاميرا
    Ref<InputEventScreenDrag> drag = event;
    if (drag.is_valid() && m_is_touching) {
        m_total_drag_dist += drag->get_relative().length();
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

void CarModeler::_on_btn_vertex_mode_pressed() { m_mode = 0; m_selected_verts.clear(); rebuild_render_mesh(); }
void CarModeler::_on_btn_edge_mode_pressed()   { m_mode = 1; m_selected_edges.clear(); rebuild_render_mesh(); }
void CarModeler::_on_btn_face_mode_pressed()   { m_mode = 2; m_selected_faces.clear(); rebuild_render_mesh(); }
void CarModeler::_on_btn_object_mode_pressed() { m_mode = 3; rebuild_render_mesh(); }

void CarModeler::_on_btn_make_face_pressed() {
    if (m_mode == 0 && m_selected_verts.size() >= 2) {
        std::vector<VertId> v_list(m_selected_verts.begin(), m_selected_verts.end());
        FaceId fid = m_bmesh.make_smart_face(v_list);
        if (fid != -1) {
            m_selected_verts.clear();
            m_selected_faces.insert(fid);
            m_mode = 2;
            rebuild_render_mesh();
            if (m_lbl_status) m_lbl_status->set_text("✨ تم إنشاء الوجه الذكي بنجاح!");
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
            if (m_lbl_status) m_lbl_status->set_text("⬆️ تم بثق الوجه بنجاح");
        }
    }
}

void CarModeler::_on_btn_delete_pressed() {
    for (FaceId f : m_selected_faces) {
        m_bmesh.faces[f].deleted = true;
    }
    m_selected_faces.clear();
    rebuild_render_mesh();
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
            // رسم وجه رباعي نظيف 100% بدون أي وتر أو خط وهمي قطري
            Vector3 v0 = pts[0], v1 = pts[1], v2 = pts[2], v3 = pts[3];

            // المثلث الأول
            st->set_normal(f.normal); st->set_color(col); st->set_uv(Vector2(0, 0)); st->add_vertex(v0);
            st->set_normal(f.normal); st->set_color(col); st->set_uv(Vector2(1, 0)); st->add_vertex(v1);
            st->set_normal(f.normal); st->set_color(col); st->set_uv(Vector2(1, 1)); st->add_vertex(v2);

            // المثلث الثاني
            st->set_normal(f.normal); st->set_color(col); st->set_uv(Vector2(0, 0)); st->add_vertex(v0);
            st->set_normal(f.normal); st->set_color(col); st->set_uv(Vector2(1, 1)); st->add_vertex(v2);
            st->set_normal(f.normal); st->set_color(col); st->set_uv(Vector2(0, 1)); st->add_vertex(v3);
        } else {
            // للأوجه غير الرباعية (Triangles أو N-Gons)
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
