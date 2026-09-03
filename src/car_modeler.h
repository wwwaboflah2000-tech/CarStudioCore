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
    int m_mode = 2;
    bool m_ctrl_active = false;

    std::set<VertId> m_selected_verts;
    std::set<EdgeId> m_selected_edges;
    std::set<FaceId> m_selected_faces;

    Camera3D* m_camera = nullptr;
    MeshInstance3D* m_car_mesh = nullptr;
    Label* m_lbl_status = nullptr;
    Button* m_btn_ctrl = nullptr;

    float m_cam_dist = 4.5f;
    float m_cam_pitch = -0.35f;
    float m_cam_yaw = 0.5f;
    bool m_is_orbiting = false;

    void update_camera_transform();
    void rebuild_render_mesh();

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
