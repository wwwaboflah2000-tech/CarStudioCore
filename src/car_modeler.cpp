#include "car_modeler.h"
#include <godot_cpp/core/class_db.hpp>
#include <godot_cpp/classes/surface_tool.hpp>
#include <godot_cpp/classes/input_event_screen_touch.hpp>
#include <godot_cpp/classes/input_event_screen_drag.hpp>
#include <algorithm>

namespace godot {

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
    m_selected_faces.insert(4);
    rebuild_render_mesh();

    if (m_lbl_status) {
        m_lbl_status->set_text("🟢 BMesh Core Active (Zero GDScript)");
    }
}

void CarModeler::update_camera_transform() {
    if (!m_camera) return;
    Vector3 target(0, 0.75f, 0);
    Basis rot = Basis::from_euler(Vector3(m_cam_pitch, m_cam_yaw, 0));
    m_camera->set_global_position(target + rot.xform(Vector3(0, 0, m_cam_dist)));
    m_camera->look_at(target, Vector3(0, 1, 0));
}

void CarModeler::_unhandled_input(const Ref<InputEvent>& event) {
    Ref<InputEventScreenTouch> touch = event;
    if (touch.is_valid()) {
        m_is_orbiting = touch->is_pressed();
        return;
    }
    Ref<InputEventScreenDrag> drag = event;
    if (drag.is_valid() && m_is_orbiting) {
        Vector2 rel = drag->get_relative();
        m_cam_yaw -= rel.x * 0.005f;
        m_cam_pitch = std::clamp(m_cam_pitch - rel.y * 0.005f, -1.4f, 1.4f);
        update_camera_transform();
    }
}

void CarModeler::_on_btn_ctrl_toggled(bool toggled_on) {
    m_ctrl_active = toggled_on;
    if (m_btn_ctrl) {
        m_btn_ctrl->set_text(m_ctrl_active ? "Ctrl: ON" : "Ctrl: OFF");
    }
}

void CarModeler::_on_btn_vertex_mode_pressed() {
    m_mode = 0;
    m_selected_verts.clear();
    rebuild_render_mesh();
}

void CarModeler::_on_btn_edge_mode_pressed() {
    m_mode = 1;
    m_selected_edges.clear();
    rebuild_render_mesh();
}

void CarModeler::_on_btn_face_mode_pressed() {
    m_mode = 2;
    m_selected_faces.clear();
    rebuild_render_mesh();
}

void CarModeler::_on_btn_object_mode_pressed() {
    m_mode = 3;
    rebuild_render_mesh();
}

void CarModeler::_on_btn_make_face_pressed() {
    if (m_mode == 0 && m_selected_verts.size() >= 2) {
        std::vector<VertId> v_list(m_selected_verts.begin(), m_selected_verts.end());
        FaceId fid = m_bmesh.make_smart_face(v_list);
        if (fid != -1) {
            m_selected_verts.clear();
            m_selected_faces.insert(fid);
            m_mode = 2;
            rebuild_render_mesh();
            if (m_lbl_status) {
                m_lbl_status->set_text("✨ تم إنشاء الوجه الذكي بنجاح!");
            }
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
            if (m_lbl_status) {
                m_lbl_status->set_text("⬆️ تم بثق الوجه بنجاح");
            }
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

        for (size_t j = 1; j < pts.size() - 1; ++j) {
            st->set_normal(f.normal); st->set_color(col); st->set_uv(Vector2(1, 0)); st->add_vertex(pts[0]);
            st->set_normal(f.normal); st->set_color(col); st->set_uv(Vector2(0, 1)); st->add_vertex(pts[j]);
            st->set_normal(f.normal); st->set_color(col); st->set_uv(Vector2(0, 0)); st->add_vertex(pts[j + 1]);
        }
    }

    m_car_mesh->set_mesh(st->commit());
}

} // namespace godot
