#include <pangolin/display/display.h>
#include <pangolin/display/view.h>      // NEW: Required for struct View
#include <pangolin/handler/handler.h>   // NEW: Required for Handler3D
#include <pangolin/gl/gldraw.h>        // NEW: Required for glDrawAxis
#include <pangolin/gl/glvbo.h>
#include <pangolin/gl/glsl.h>
#include <Eigen/Core>
#include <vector>
#include <random>

// Annotated Shader
const std::string pcd_shader = R"Shader(
@start vertex
#version 120
attribute vec3 a_position;
varying float v_z;
void main() {
    gl_Position = gl_ModelViewProjectionMatrix * vec4(a_position, 1.0);
    v_z = a_position.z;
}

@start fragment
#version 120
varying float v_z;
void main() {
    float intensity = clamp((v_z + 1.0) / 2.0, 0.0, 1.0);
    gl_FragColor = vec4(0.0, intensity, 1.0 - intensity, 1.0); 
}
)Shader";

int main() {
    pangolin::CreateWindowAndBind("Pango GPU PCD Shader", 1024, 768);
    glEnable(GL_DEPTH_TEST);

    // 1. Setup Camera
    pangolin::OpenGlRenderState s_cam(
        pangolin::ProjectionMatrix(1024, 768, 420, 420, 512, 384, 0.1, 1000),
        pangolin::ModelViewLookAt(-2, -2, -5, 0, 0, 0, pangolin::AxisY)
    );
    
    // 2. Setup View and Handler
    pangolin::Handler3D handler(s_cam);
    pangolin::View& d_cam = pangolin::CreateDisplay()
        .SetBounds(0.0, 1.0, 0.0, 1.0, -1024.0f/768.0f)
        .SetHandler(&handler);

    // 3. Prepare Shader
    pangolin::GlSlProgram prog;
    prog.AddShader(pangolin::GlSlAnnotatedShader, pcd_shader);
    prog.Link();

    // 4. Initialize VBO
    const int num_points = 200000;
    std::vector<Eigen::Vector3f> points(num_points);
    pangolin::GlBuffer vbo(pangolin::GlArrayBuffer, points, GL_DYNAMIC_DRAW);

    std::default_random_engine gen;
    std::uniform_real_distribution<float> dist(-2.0f, 2.0f);

    while(!pangolin::ShouldQuit()) {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        
        // Activate display
        d_cam.Activate(s_cam);

        // Update data
        for(auto& p : points) p = {dist(gen), dist(gen), dist(gen)};
        vbo.Upload(points.data(), sizeof(float) * 3 * num_points);

        // Render
        prog.Bind();
        glPointSize(2.0f);
        pangolin::RenderVbo(vbo, GL_POINTS);
        prog.Unbind();

        pangolin::glDrawAxis(1.0f);

        pangolin::FinishFrame();
    }
    return 0;
}