#ifdef _WIN32
#pragma warning(disable:4305) // truncating double literal to float
#endif
#include <iostream>
#include "gl-util/glfw-util.h"
#include "util/stopwatch.h"
#include "util/quat.h"
#include "sim/scene.h"

#ifdef USE_OVR
#include "vr/ovr-util.h"
#endif

static void LogGLFWError(int code, const char *message) {
  std::cerr << "glfw error " << code << ": " << message << std::endl;
}

static glfw::Window* window;
static size_t frame_idx;

static void KeyCallback(
    GLFWwindow* w, int key, int scancode, int action, int mods) {
  if (action == GLFW_PRESS) {
    if (key == GLFW_KEY_ESCAPE) {
      window->SetShouldClose();
    }
  }
}

static void ScrollCallback(GLFWwindow *win, double dx, double dy) {
  
}

static void MouseButtonCallback(
    GLFWwindow *win, int button, int action, int mods) {
  
}

static void CursorPosCallback(GLFWwindow *w, double x, double y) {
  
}

static void UpdateFPS() {
  static size_t last_update_frame = 0;
  static Stopwatch fps_stopwatch;
  const size_t period_frames = 10;
  const double period_seconds = .5;
  if (frame_idx - last_update_frame >= period_frames ||
      fps_stopwatch.TimeSinceRestart() > period_seconds) {
    double fps = (frame_idx - last_update_frame)
      / fps_stopwatch.Restart();
    last_update_frame = frame_idx;
    window->SetTitle("FPS: " + std::to_string(fps));
  }
}

// Order of keys is: forward, backwards, left, right, up, down (-z, +z, -x, +x, +y, -y), ALL CAPS.
dvec3 ThreeDofInput(glfw::Window& w, const char* keys) {
  dvec3 r = {0, 0, 0};
  if (w.IsKeyPressed(keys[2]))
    r.x -= 1;
  if (w.IsKeyPressed(keys[3]))
    r.x += 1;
  if (w.IsKeyPressed(keys[0]))
    r.z -= 1;
  if (w.IsKeyPressed(keys[1]))
    r.z += 1;
  if (w.IsKeyPressed(keys[4]))
    r.y += 1;
  if (w.IsKeyPressed(keys[5]))
    r.y -= 1;
  return r;
}

double rnd() {
  return (((rand()+.5)/(RAND_MAX+1.)+rand())/(RAND_MAX+1.)+rand())/(RAND_MAX+1.);
}

int main() {
  try {
#ifdef USE_OVR
    ovr::Session vr;
    if (!vr.Init()) return 1;
#endif

    glfwSetErrorCallback(&LogGLFWError);
    glfw::Initializer glfw_init;
    glfw::Window win_(ivec2(0, 20), ivec2(900, 900), "hello world", false);
    ::window = &win_;
    window->MakeCurrent();
    GL::InitGl3wIfNeeded();
    GL::LogInfo();

#ifdef USE_OVR
    glfwSwapInterval(0);
    if (!vr.InitSwapChains()) return 1;
#endif

    window->SetKeyCallback(&KeyCallback);
    window->SetScrollCallback(&ScrollCallback);
    window->SetMouseButtonCallback(&MouseButtonCallback);
    window->SetCursorPosCallback(&CursorPosCallback);

    Scene scene;

    struct Manipulator {
      size_t vert1;
      size_t vert2;
      dvec3 pos1;
      dvec3 vel1;
      dvec3 angular_velocity;
      dvec3 dir; // from vert1 to vert2
      double homing_speed = .2;
      double angular_homing_speed = .2;
    };
    Manipulator manip;
    manip.vert1 = 0;
    manip.vert2 = 1;
    manip.dir = dvec3(1, 0, 0);
    scene.external_constraints.resize(5);
    for (int i = 0; i < 3; ++i) scene.external_constraints[i].vert_idx = manip.vert1;
    for (int i = 3; i < 5; ++i) scene.external_constraints[i].vert_idx = manip.vert2;
    scene.external_constraints[0].axis = dvec3(1, 0, 0);
    scene.external_constraints[1].axis = dvec3(0, 1, 0);
    scene.external_constraints[2].axis = dvec3(0, 0, 1);

    auto reset = [&] {
                   scene.rope.Create(2);
#ifdef USE_OVR
                   dvec3 d = manip.pos1 - scene.rope.verts[0].pos;
                   for (auto& v : scene.rope.verts) v.pos += d;
#else
                   manip.pos1 = scene.rope.verts[0].pos;
                   manip.vel1 = dvec3();
#endif
                 };

    reset();
    //scene.gravity = dvec3(0, -9.8, 0);
    scene.gravity = dvec3(0, -.98, 0);

    Camera camera;
    camera.pos = fvec3(-1.5, .24, 1.);
    camera.LookAt((scene.rope.verts[0].pos+scene.rope.verts[std::min((int)scene.rope.verts.size() - 1, 30)].pos*2)/3);

    Stopwatch frame_stopwatch;
    while (!window->ShouldClose()) {
      double dt = frame_stopwatch.Restart();
      dt = std::min(dt, 1.);

      UpdateFPS();

      if (window->IsKeyPressed(GLFW_KEY_R)) reset();

#ifdef USE_OVR
      ovr::FrameInfo frame;
      if (!vr.BeginFrame(frame_idx, frame)) break;
      manip.pos1 = frame.hands[1].pose.position;
      manip.vel1 = frame.hands[1].pose.velocity;
      if (frame_idx == 0) reset(); // move rope end to initial hand position
#else
      dvec3 in = ThreeDofInput(*window, "IKJLUM");
      manip.pos1 += manip.vel1 * dt;
      manip.vel1 = in * 1e0;
#endif

      // Form constraints for the manipulator.
      dvec3 rope_dir = scene.rope.verts[1].pos - scene.rope.verts[0].pos;
      dvec3 axis_a = rope_dir.ArbitraryPerpendicular().Normalized();
      dvec3 axis_b = rope_dir.Cross(axis_a).Normalized();
      scene.external_constraints[3].axis = axis_a;
      scene.external_constraints[4].axis = axis_b;
      for (int i = 0; i < 5; ++i) scene.external_constraints[i].speed = scene.external_constraints[i].axis.Dot(manip.vel1);
      dvec3 vel2 = manip.angular_velocity.Cross(rope_dir);
      scene.external_constraints[3].speed += axis_a.Dot(vel2);
      scene.external_constraints[4].speed += axis_b.Dot(vel2);
      // If rope somehow got away from the manipulator, pull it towards the manipulator at constant speed.
      {
        dvec3 d = manip.pos1 - scene.rope.verts[manip.vert1].pos;
        double l = d.Length();
        if (l > 1e-6) {
          // Pull the vert towards the manipulator at constant speed.
          double c = std::min(l/dt, manip.homing_speed)/l;
          for (int i = 0; i < 5; ++i) scene.external_constraints[i].speed += scene.external_constraints[i].axis.Dot(d)*c;
        }
      }
      // Similarly pull rope's orientation towards manipulator's orientation.
      // todo: Uncomment after testing that the angular velocity constraint works.
      /*
      {
        double ang = acos(manip.dir.Dot(rope_dir.Normalized()));
        std::cerr << ang << std::endl;
        if (fabs(ang) > 1e-6) {
          dvec3 ang_vel = -manip.dir.Cross(rope_dir).Normalized() * std::min(fabs(ang)/dt, manip.angular_homing_speed);
          dvec3 add_vel2 = ang_vel.Cross(rope_dir);
          scene.external_constraints[3].speed += axis_a.Dot(add_vel2);
          scene.external_constraints[4].speed += axis_b.Dot(add_vel2);
        }
      }//*/

      const int substeps = 1;
      for (int i = 0; i < substeps; ++i) scene.PhysicsStep(dt/substeps);

      if (frame_idx % 120 == 0) scene.PrintStats();

#ifdef USE_OVR
      for (size_t ei = 0; ei < 2; ++ei) {
        vr.SetEyeRenderTarget(ei);
        scene.Render(frame.eyes[ei].view_projection);
        vr.CommitEye(ei);
      }
      vr.EndFrame(frame);
#else
      scene.Render(camera.ViewProjection());
#endif
      
      window->SwapBuffers();
      glfwPollEvents();
      ++frame_idx;
    }

#ifdef USE_OVR
    vr.DestroySwapChains();
#endif
  } catch (std::exception& e) {
    std::cerr << "exception: " << e.what() << std::endl;
    return 1;
  }

  return 0;
}
