#ifndef CAR_MODELER_H
#define CAR_MODELER_H

#include <godot_cpp/classes/node3d.hpp>
#include <godot_cpp/classes/camera3d.hpp>
#include <godot_cpp/classes/mesh_instance3d.hpp>
#include <godot_cpp/classes/label.hpp>
#include <godot_cpp/classes/button.hpp>
#include <godot_cpp/classes/input_event.hpp>
#include "bmesh.h"
#include <set>

namespace godot {

class CarModeler : public Node3D {
    GDCLASS(CarModeler, Node3D);

private:
    BMesh m_bmesh;
    int m_mode = 2; // 0: Vertex, 1: Edge, 2: Face, 3: Object
    bool m_ctrl_active = false;

    std::set<VertId> m_selected_verts;
    std::set<EdgeId> m_selected_edges;
    std::set<FaceId> m_selected_faces;

    Camera3D* m_camera = nullptr;
    MeshInstance3D* m_car_mesh = nullptr;
    MeshInstance3D* m_overlay_mesh = nullptr; // طبقة إظهار النقاط والحواف المحددة
    Node3D* m_gizmo_root = nullptr;
    Label* m_lbl_status = nullptr;
    Button* m_btn_ctrl = nullptr;

    // تتبع الكاميرا واللمس
    float m_cam_dist = 4.5f;
    float m_cam_pitch = -0.35f;
    float m_cam_yaw = 0.5f;
    bool m_is_touching = false;
    Vector2 m_touch_start_pos;
    float m_total_drag_dist = 0.0f;

    // متغيرات الجزمو الديناميكي
    Vector3 m_gizmo_pos = Vector3(0, 0, 0);
    int m_active_gizmo_axis = -1;
    bool m_is_dragging_gizmo = false;
    Vector3 m_gizmo_prev_hit;

    void update_camera_transform();
    void rebuild_render_mesh();
    void setup_gizmo_nodes();
    void update_gizmo();

    VertId pick_vertex_at_screen_pos(const Vector2& screen_pos);
    EdgeId pick_edge_at_screen_pos(const Vector2& screen_pos);
    FaceId pick_face_at_screen_pos(const Vector2& screen_pos);
    int pick_gizmo_axis(const Vector2& screen_pos);

    Vector3 get_ray_plane_intersection(const Vector3& ray_origin, const Vector3& ray_dir, const Vector3& plane_point, const Vector3& plane_normal);

protected:
    static void _bind_methods();

public:
    CarModeler();
    ~CarModeler();

    void _ready() override;
    void _unhandled_input(const Ref<InputEvent>& event) override;

    void _on_btn_vertex_mode_pressed();
    void _on_btn_edge_mode_pressed();
    void _on_btn_face_mode_pressed();
    void _on_btn_object_mode_pressed();
    void _on_btn_ctrl_toggled(bool toggled_on);
    void _on_btn_make_face_pressed();
    void _on_btn_extrude_pressed();
    void _on_btn_delete_pressed();
};

} // namespace godot

#endif
