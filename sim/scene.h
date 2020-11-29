#pragma once
#include "util/vec.h"
#include "util/quat.h"
#include "gl-util/gl-common.h"
#include "gl-util/vertex-array.h"
#include "gl-util/shader.h"
#include <vector>
#include <list>
#include <deque>

struct Rope {
  struct Vert {
    dvec3 pos;
    dvec3 vel;
    double mass;
    double target_length; // of the segment between this vert and the next
  };

  double density = 1e3;
  double radius = 1e-2;
  double max_segment_length = 5e-2;

  std::vector<Vert> verts;

  void Create(size_t n);
};

struct Camera {
 public:
  fvec3 pos = fvec3(0, 0, 0);
  float yaw = 0; // counterclockwise from -z axis
  float pitch = 0; // upwards
  float fov = (float)M_PI/2;
  float aspect_ratio = 1;

  void LookAt(fvec3 p);
  fmat4 ViewProjection() const;
};

struct RopeConstraint {
  size_t vert_idx;
  dvec3 vel;

  bool satisfied; // assigned by PhysicsStep()
};

struct Scene {
  Scene();

  void Render(fmat4 view_projection);
  void PhysicsStep(double dt);
  void PrintStats() const;

  Rope rope;
  dvec3 gravity = dvec3(0, 0, 0);
  // When segment length deviates from target length (because of numerical errors),
  // we force the segment to contract at rate target_len*(len/target_len-1)*segment_length_restoration_fudge_factor.
  // The units are 1/second.
  double segment_length_restoration_fudge_factor = 3;

  std::vector<RopeConstraint> external_constraints;

  fvec3 light_vec = fvec3(-3, 2, 1).Normalized(); // direction from which the light is coming

  GL::Shader rope_shader;
  std::unique_ptr<GL::VertexArray> rope_segment_mesh;
};
