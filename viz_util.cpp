#include <pangolin/display/display.h>
#include <pangolin/display/view.h>
#include <pangolin/handler/handler.h>
#include <pangolin/gl/gldraw.h>
#include <pangolin/gl/glvbo.h>
#include <pangolin/gl/glsl.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <mutex>
#include <thread>
#include <vector>
#include <sys/mman.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>

// Struct for interleaved XYZ + RGB data
struct Point6D { 
    float x, y, z; 
    float r, g, b; 
};

// Shader for hardware-accelerated color rendering
const std::string color_shader = R"Shader(
@start vertex
#version 120
attribute vec3 a_position;
attribute vec3 a_color;
varying vec3 v_color;
void main() {
    gl_Position = gl_ModelViewProjectionMatrix * vec4(a_position, 1.0);
    v_color = a_color;
}
@start fragment
#version 120
varying vec3 v_color;
void main() {
    gl_FragColor = vec4(v_color, 1.0);
}
)Shader";

class StandaloneViz {
public:
    std::vector<Point6D> cloud;
    std::vector<Eigen::Vector3f> traj;
    std::vector<Eigen::Matrix4f> keyframes;
    std::mutex data_mtx;
    bool running = true;

    void Run() {
        pangolin::CreateWindowAndBind("SLAM SHM Visualizer", 1024, 768);
        glEnable(GL_DEPTH_TEST);

        pangolin::OpenGlRenderState s_cam(
            pangolin::ProjectionMatrix(1024, 768, 420, 420, 512, 384, 0.1, 1000),
            pangolin::ModelViewLookAt(-2, -2, -5, 0, 0, 0, pangolin::AxisY)
        );
        
        pangolin::View& d_cam = pangolin::CreateDisplay()
            .SetBounds(0.0, 1.0, 0.0, 1.0, -1024.0f/768.0f)
            .SetHandler(new pangolin::Handler3D(s_cam));

        pangolin::GlSlProgram prog;
        prog.AddShader(pangolin::GlSlAnnotatedShader, color_shader);
        prog.Link();

        while(!pangolin::ShouldQuit()) {
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
            d_cam.Activate(s_cam);
            pangolin::glDrawAxis(1.0f);

            {
                std::lock_guard<std::mutex> lock(data_mtx);

                // 1. PCD Rendering
                if(!cloud.empty()) {
                    prog.Bind();
                    glEnableClientState(GL_VERTEX_ARRAY);
                    glEnableClientState(GL_COLOR_ARRAY);
                    glVertexPointer(3, GL_FLOAT, sizeof(Point6D), &cloud[0].x);
                    glColorPointer(3, GL_FLOAT, sizeof(Point6D), &cloud[0].r);
                    glDrawArrays(GL_POINTS, 0, cloud.size());
                    glDisableClientState(GL_COLOR_ARRAY);
                    glDisableClientState(GL_VERTEX_ARRAY);
                    prog.Unbind();
                }

                // 2. Trajectory Rendering
                if(traj.size() > 1) {
                    glLineWidth(2.0f);
                    glColor3f(1.0f, 0.0f, 0.0f);
                    pangolin::glDrawLineStrip(traj);
                }

                // 3. Keyframe Rendering
                for(const auto& T : keyframes) {
                    pangolin::glDrawAxis(T, 0.2f);
                }
            }
            pangolin::FinishFrame();
        }
        running = false;
    }

    void SHMListener() {
        const char* shm_path = "/dev/shm/slam_buffer";
        int fd = -1;

        // --- STAGE 1: Wait for Python to create the file ---
        std::cout << "Waiting for SHM file: " << shm_path << " ..." << std::endl;
        while (running && fd < 0) {
            fd = open(shm_path, O_RDONLY);
            if (fd < 0) {
                usleep(500000); // Check every 0.5 seconds
            }
        }
        
        if (!running) return;
        std::cout << "SHM Connected!" << std::endl;
    // --- STAGE 2: Map the memory ---
        struct stat st;
        fstat(fd, &st);
        void* addr = mmap(NULL, st.st_size, PROT_READ, MAP_SHARED, fd, 0);
        int32_t* headers = (int32_t*)addr;

        // --- STAGE 3: Continuous Monitoring ---
        while(running) {
            // Read headers for sizes
            int32_t num_pcd  = headers[0];
            int32_t num_traj = headers[1];
            int32_t num_kf   = headers[2];

            if (num_pcd > 0 || num_traj > 0 || num_kf > 0) {
                std::lock_guard<std::mutex> lock(data_mtx);
                
                // Point Cloud (Offset 100 bytes)
                Point6D* pcd_ptr = (Point6D*)((char*)addr + 100);
                cloud.assign(pcd_ptr, pcd_ptr + num_pcd);

                // Trajectory (Follows PCD)
                Eigen::Vector3f* traj_ptr = (Eigen::Vector3f*)((char*)pcd_ptr + (num_pcd * sizeof(Point6D)));
                traj.assign(traj_ptr, traj_ptr + num_traj);

                // Keyframes (Follows Trajectory)
                float* kf_ptr = (float*)((char*)traj_ptr + (num_traj * sizeof(Eigen::Vector3f)));
                keyframes.clear();
                for(int i=0; i<num_kf; ++i) {
                    Eigen::Map<Eigen::Matrix4f> T_map(kf_ptr + (i*16));
                    keyframes.push_back(T_map);
                }
            }
            // 60Hz update rate to match typical display
            usleep(16666); 
        }
        munmap(addr, st.st_size);
        close(fd);
    }
};

int main() {
    StandaloneViz viz;
    std::thread t(&StandaloneViz::SHMListener, &viz);
    viz.Run();
    if(t.joinable()) t.join();
    return 0;
}