#ifndef PEELMESH_WINDOW_H
#define PEELMESH_WINDOW_H

#include <Open3D/Open3D.h>

#include <peelmesh/pipeline.hpp>
#include <format>
#include <thread>
#include <filesystem>

#include "portable-file-dialogs.h"

namespace rendering = open3d::visualization::rendering;
namespace gui = open3d::visualization::gui;

const int OPEN_FILE = 1;
const int EXPORT_SEGMENTS = 2;

namespace peelmesh
{
    class SceneWidget : public gui::SceneWidget
    {
    public:
        std::shared_ptr<peelmesh::PeelMeshPipeline> pipe_;
        std::vector<int> landmarks;
        std::vector<std::string> landmark_names;
        std::vector<std::string> geodesic_names;
        std::vector<std::shared_ptr<open3d::geometry::Geometry3D>> geodesics_;
        std::shared_ptr<open3d::geometry::LineSet> geodesic;

        std::vector<std::shared_ptr<open3d::geometry::Geometry3D>> segments_;
        std::vector<std::string> segment_names;

        std::vector<std::vector<int>> boundaries;

        gui::Widget::EventResult Mouse(const gui::MouseEvent &e) override
        {
            if (e.type == gui::MouseEvent::BUTTON_DOWN && e.button.button == gui::MouseButton::LEFT && e.modifiers == int(gui::KeyModifier::CTRL))
            {
                if (!pipe_)
                    return gui::Widget::EventResult::IGNORED;

                auto depth_func = [&](std::shared_ptr<open3d::geometry::Image> depth_img)
                {
                    auto x = e.x - this->GetFrame().x;
                    auto y = e.y - this->GetFrame().y;
                    // auto z = this->GetRenderer().GetCamera().GetZNear();
                    auto depth = depth_img->FloatValueAt(x, y);

                    const auto &[index, coord] = PickVertex(depth_img, x, y);

                    if (index != -1)
                    {
                        auto sphere = open3d::geometry::TriangleMesh::CreateSphere(0.1);
                        sphere->PaintUniformColor({1, 0, 0});
                        sphere->Translate({coord[0], coord[1], coord[2]});
                        landmark_names.push_back("sphere" + std::to_string(landmark_names.size() - 1));
                        this->GetScene()->AddGeometry(landmark_names.back(), sphere.get(), {});
                        landmarks.push_back(index);

                        if (landmarks.size() > 1)
                        {
                            geodesics_.push_back(geodesic);
                            geodesic_names.push_back("geodesic" + std::to_string(geodesics_.size() - 1));
                            this->GetScene()->AddGeometry(geodesic_names.back(), geodesic.get(), {});
                        }
                    }
                };

                this->GetScene()->GetScene()->RenderToDepthImage(depth_func);
                return gui::Widget::EventResult::CONSUMED;
            }
            else if (e.type == gui::MouseEvent::MOVE && landmarks.size() > 0)
            {
                auto func = [&](std::shared_ptr<open3d::geometry::Image> depth_img)
                {
                    auto start = landmarks.back();

                    const auto &[index, coord] = PickVertex(depth_img, e.x - this->GetFrame().x, e.y - this->GetFrame().y);

                    if (index == -1 || index == start)
                        return;

                    auto func2 = [&]()
                    {
                        auto geodesic_pts = pipe_->ComputeGeodesic(start, index);
                        if (geodesic_pts.empty())
                            return;
                        auto line = std::make_shared<open3d::geometry::LineSet>();
                        line->points_ = geodesic_pts;
                        for (int i = 0; i < line->points_.size() - 1; i++)
                        {
                            line->lines_.push_back({i, i + 1});
                        }
                        line->PaintUniformColor({0, 1, 0});

                        geodesic = line;
                        this->GetScene()->RemoveGeometry("geodesic_tmp");

                        rendering::MaterialRecord geo_mat;
                        geo_mat.shader = "unlitLine";
                        geo_mat.line_width = 3.0f;

                        this->GetScene()->AddGeometry("geodesic_tmp", line.get(), geo_mat);
                    };

                    func2();
                };

                this->GetScene()->GetScene()->RenderToDepthImage(func);
                return gui::Widget::EventResult::CONSUMED;
            }
            else if (e.type == gui::MouseEvent::BUTTON_DOWN && e.button.button == gui::MouseButton::RIGHT && e.modifiers == int(gui::KeyModifier::CTRL))
            {
                if (landmarks.size() > 0)
                {
                    this->GetScene()->RemoveGeometry(landmark_names.back());
                    landmark_names.pop_back();
                    landmarks.pop_back();
                }
                if (geodesics_.size() > 0)
                {
                    this->GetScene()->RemoveGeometry(geodesic_names.back());
                    geodesic_names.pop_back();
                    geodesics_.pop_back();
                }

                if (landmarks.empty())
                {
                    this->GetScene()->RemoveGeometry("geodesic_tmp");
                }
                return gui::Widget::EventResult::CONSUMED;
            }
            else if (e.type == gui::MouseEvent::BUTTON_DOWN && e.button.button == gui::MouseButton::RIGHT)
            {
                boundaries.push_back(landmarks);
                landmarks.clear();
                this->GetScene()->RemoveGeometry("geodesic_tmp");
            }
            return gui::SceneWidget::Mouse(e);
        }

        gui::Widget::EventResult Key(const gui::KeyEvent &e) override
        {
            if (e.key == gui::KeyName::KEY_R && e.type == gui::KeyEvent::DOWN)
            {
                auto aabb = this->GetScene()->GetBoundingBox();
                this->SetupCamera(60.0, aabb, aabb.GetCenter().cast<float>());
                return gui::Widget::EventResult::CONSUMED;
            }
            else if (e.key == gui::KeyName::KEY_ENTER && e.type == gui::KeyEvent::DOWN)
            {
                std::vector<std::shared_ptr<TriangleMesh>> segments;
                if (!boundaries.empty())
                {
                    for (auto &boundary : boundaries)
                    {
                        if (boundary.size() < 3)
                            break;

                        if (boundary.front() != boundary.back())
                            boundary.push_back(boundary.front());

                        for (int i = 0; i < boundary.size() - 1; i++)
                        {
                            pipe_->AddGeodesicPath(boundary[i], boundary[i + 1]);
                        }

                        segments.push_back(pipe_->PeelOffMesh(boundary));
                    }
                }

                for (auto &segment : segments)
                {
                    const auto &[verts, tris] = segment->getMeshData();
                    auto mesh = std::make_shared<open3d::geometry::TriangleMesh>(verts, tris);

                    mesh->PaintUniformColor({rand() % 255 / 255.0, rand() % 255 / 255.0, rand() % 255 / 255.0});
                    mesh->ComputeVertexNormals();
                    for (int i = 0; i < mesh->vertices_.size(); i++)
                    {
                        mesh->vertices_[i] += 0.02 * mesh->vertex_normals_[i];
                    }

                    segments_.push_back(mesh);
                    segment_names.push_back("segment" + std::to_string(segments_.size() - 1));
                    this->GetScene()->AddGeometry(segment_names.back(), mesh.get(), {});
                }
            }
            return gui::SceneWidget::Key(e);
        }

        std::pair<int, Eigen::Vector3d>
        PickVertex(std::shared_ptr<open3d::geometry::Image> depth_img, int u, int v)
        {
            auto z = depth_img->FloatValueAt(u, v).second;
            if (z == 1.0)
                return {-1, Eigen::Vector3d::Zero()};

            auto world_coord = this->GetScene()->GetCamera()->Unproject(u, v, z, this->GetFrame().width, this->GetFrame().height);

            std::vector<int> indicies;
            std::vector<double> distances2;
            pipe_->GetMesh()->SearchKNN(world_coord.cast<double>(), 1, indicies, distances2);

            auto coord = pipe_->GetMesh()->vertices[indicies[0]].position;

            return {indicies[0], coord};
        }

        void ResetToDefault()
        {
            this->GetScene()->ClearGeometry();
            landmarks.clear();
            landmark_names.clear();
            geodesics_.clear();
            geodesic_names.clear();
            boundaries.clear();
            segments_.clear();
            segment_names.clear();
        }
    };

    class Window : public gui::Window
    {
    public:
        std::shared_ptr<SceneWidget> main_scene_;

        std::shared_ptr<peelmesh::PeelMeshPipeline> pipe_;

        float scale_factor = 1.0f;
        Eigen::Vector3d rotation_center = Eigen::Vector3d::Zero();

        rendering::MaterialRecord wire_mat;

        std::chrono::steady_clock::time_point last_time = std::chrono::high_resolution_clock::now();
        unsigned int frame_count_ = 0;

        Window(const std::string &title, int width, int height)
            : gui::Window(title, width, height)
        {
            main_scene_ = std::make_shared<SceneWidget>();
            main_scene_->SetScene(std::make_shared<rendering::Open3DScene>(this->GetRenderer()));

            this->AddChild(main_scene_);

            DrawPipeline();
        }
        virtual ~Window() {}

        void Layout(const gui::LayoutContext &context) override
        {
            auto window_rect = this->GetContentRect();
            main_scene_->SetFrame(window_rect);
            main_scene_->Layout(context);
        }

        void DrawPipeline()
        {
            if (!pipe_)
                return;

            const auto &[verts, tris] = pipe_->GetMeshData();
            auto mesh = std::make_shared<open3d::geometry::TriangleMesh>(verts, tris);
            mesh->ComputeVertexNormals();
            main_scene_->GetScene()->RemoveGeometry("mesh");
            main_scene_->GetScene()->AddGeometry("mesh", mesh.get(), {});

            auto wire = open3d::geometry::LineSet::CreateFromTriangleMesh(*mesh);
            wire->PaintUniformColor({1, 1, 1});
            main_scene_->GetScene()->RemoveGeometry("wire");
            main_scene_->GetScene()->AddGeometry("wire", wire.get(), wire_mat);

            static bool first = true;
            if (first)
            {
                ResetCamera();
                first = false;
            }
        }

        void ResetCamera()
        {
            auto aabb = main_scene_->GetScene()->GetBoundingBox();
            main_scene_->SetupCamera(60.0, aabb, aabb.GetCenter().cast<float>());
        }

        virtual void OnMenuItemSelected(gui::Menu::ItemId item_id) override
        {
            switch (item_id)
            {
            case OPEN_FILE:
            {
                auto func = [&, this]()
                {
                    auto f = pfd::open_file("Open File", ".", {"Mesh Files", "*.obj *.stl *.ply *.off"}, pfd::opt::none);

                    if (f.result().empty())
                    {
                        std::cout << "No file is choosen!" << std::endl;
                        return;
                    }

                    auto mesh = open3d::io::CreateMeshFromFile(f.result()[0]);
                    mesh->RemoveDuplicatedTriangles();
                    mesh->RemoveDuplicatedVertices();

                    auto max_extent = mesh->GetAxisAlignedBoundingBox().GetMaxExtent();
                    scale_factor = 50.0f / max_extent;
                    rotation_center = mesh->GetCenter();

                    mesh->Scale(scale_factor, rotation_center);

                    pipe_ = std::make_shared<peelmesh::PeelMeshPipeline>(mesh->vertices_, mesh->triangles_);

                    main_scene_->pipe_ = pipe_;

                    gui::Application::GetInstance().PostToMainThread(this, [this]()
                                                                     {
                                                                        main_scene_->ResetToDefault();
                    DrawPipeline(); });
                };

                std::thread open_mesh_t(func);
                open_mesh_t.join();

                break;
            };
            case EXPORT_SEGMENTS:
            {
                auto func = [&, this]()
                {
                    auto export_dir = pfd::select_folder::select_folder("Choose Directory to Export Segments", pfd::path::home(), pfd::opt::none);

                    if (export_dir.result().empty())
                    {
                        std::cout << "Cancel export!" << std::endl;
                        return;
                    }

                    auto dir_path = std::filesystem::path(export_dir.result());
                    if (!std::filesystem::exists(dir_path / "segments"))
                    {
                        std::filesystem::create_directory(dir_path / "segments");
                    }

                    for (int i = 0; i < main_scene_->segments_.size(); i++)
                    {
                        auto mesh = std::dynamic_pointer_cast<open3d::geometry::TriangleMesh>(main_scene_->segments_[i]);

                        mesh->Scale(1.0 / scale_factor, rotation_center);
                        auto filename = dir_path / "segments" / ("seg_" + std::to_string(i) + ".obj");
                        bool flag = open3d::io::WriteTriangleMesh(filename.generic_string(), *mesh);
                        if (!flag)
                        {
                            std::cout << "Export " << filename.generic_string() << " failed!" << std::endl;
                        }
                    }

                    pfd::message("PeelMesh Export Segments", "Export Segments Done!", pfd::choice::ok, pfd::icon::info);
                };

                std::thread export_t(func);
                export_t.join();

                break;
            }
            default:
                break;
            }
        }
    };
    class DemoApplication
    {
        std::weak_ptr<Window> window_;
        gui::Application &instance;

    public:
        DemoApplication(std::string_view resource_path)
            : instance(gui::Application::GetInstance())
        {
            open3d::utility::SetVerbosityLevel(open3d::utility::VerbosityLevel::Error);
            instance.Initialize(resource_path.data());

            auto win = std::make_shared<Window>("PeelMesh Demo", 1920, 1080);
            window_ = win;

            instance.AddWindow(win);

            auto menubar = std::make_shared<gui::Menu>();

            auto file_menu = std::make_shared<gui::Menu>();
            file_menu->AddItem("Open", OPEN_FILE);
            menubar->AddMenu("File", file_menu);

            auto export_menu = std::make_shared<gui::Menu>();
            export_menu->AddItem("Export Segments", EXPORT_SEGMENTS);
            menubar->AddMenu("Export", export_menu);
            instance.SetMenubar(menubar);
        }

        void Run() { instance.Run(); }
    };
}

#endif /* PEELMESH_WINDOW_H */
