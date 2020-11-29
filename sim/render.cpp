#pragma warning(disable:4305) // converting double literal to float
#pragma warning(disable:4244) // converting double to float
#pragma warning(disable:4267) // implicit conversions between integer types
#include "sim/scene.h"
#include "gl-util/gl-common.h"
using namespace std;

void Camera::LookAt(fvec3 p) {
  p -= pos;
  yaw = atan2(-p.x, -p.z);
  pitch = asin(p.y / p.Length());
}

fmat4 Camera::ViewProjection() const {
  // Right to left:
  //  1. translate,
  //  2. yaw,
  //  3. pitch,
  //  4. project.
  return
    fmat4::PerspectiveProjection(fov, aspect_ratio, 1e-3, 0) *
    fmat4::RotationX(-pitch) *
    fmat4::RotationY(-yaw) *
    fmat4::Translation(-pos);
}

static const char* vertex_shader = R"(
  #version 330 core
  layout(location = 0) in vec3 vert_pos;
  layout(location = 1) in vec3 vert_normal;
  layout(location = 2) in vec3 vert_color;
  uniform mat4 model_mat;
  uniform mat4 view_proj_mat;
  out vec3 normal;
  out vec3 color;
  void main(){
    vec4 p = model_mat * vec4(vert_pos, 1);
    normal = (model_mat * vec4(vert_normal, 0)).xyz;
    color = vert_color;
    gl_Position =  view_proj_mat * p;
  }
)";

static const char* fragment_shader = R"(
  #version 330 core
  uniform vec3 light_vec;
  in vec3 normal;
  in vec3 color;
  out vec3 frag_color;
  void main(){
    frag_color = color * (.05 + max(0.f, dot(light_vec, normalize(normal))));
  }
)";

// radius 1 (in xz plane), height 2 (-1 <= y <= 1);
static std::unique_ptr<GL::VertexArray> MakeCylinderMesh(size_t sides) {
  struct Vert {
    fvec3 pos;
    fvec3 normal;
    fvec3 color = fvec3(1, 1, 1);

    Vert(fvec3 p, fvec3 n) : pos(p), normal(n) {}
  };

  std::vector<Vert> verts;
  verts.reserve(sides * 12);
  fvec3 c1(0, -1, 0), c2(0, 1, 0); // centers of the two end caps
  for (size_t i = 0; i < sides; ++i) {
    float ang1 = 2*M_PI*i/sides;
    float ang2 = 2*M_PI*(i+1)/sides;
    fvec3 a1(cosf(ang1), 0, sinf(ang1));
    fvec3 a2(cosf(ang2), 0, sinf(ang2));

    // side wall
    verts.emplace_back(c1+a1, a1);
    verts.emplace_back(c1+a2, a2);
    verts.emplace_back(c2+a2, a2);
    verts.emplace_back(c2+a2, a2);
    verts.emplace_back(c2+a1, a1);
    verts.emplace_back(c1+a1, a1);

    // end caps
    verts.emplace_back(c1   , c1);
    verts.emplace_back(c1+a2, c1);
    verts.emplace_back(c1+a1, c1);
    verts.emplace_back(c2   , c2);
    verts.emplace_back(c2+a1, c2);
    verts.emplace_back(c2+a2, c2);
  }

  auto vao = std::make_unique<GL::VertexArray>(verts.size());
  using Attribute = GL::VertexArray::Attribute;
  vector<Attribute> attrs = {
    Attribute(0, 3, GL_FLOAT, sizeof(Vert), offsetof(Vert, pos)),
    Attribute(1, 3, GL_FLOAT, sizeof(Vert), offsetof(Vert, normal)),
    Attribute(2, 3, GL_FLOAT, sizeof(Vert), offsetof(Vert, color)),
  };
  vao->AddAttributes(attrs.size(), &attrs[0], verts.size() * sizeof(verts[0]), &verts[0]);

  return vao;
}

Scene::Scene():
    rope_shader("rope_vert", "rope_frag", vertex_shader, fragment_shader),
    rope_segment_mesh(MakeCylinderMesh(50)) {}

void Scene::Render(fmat4 view_projection) {
  glClearColor(.5, .5, 1, 0);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glEnable(GL_DEPTH_TEST);
  glDepthFunc(GL_LESS);
  //glEnable(GL_CULL_FACE);
  rope_shader.Use();
  rope_shader.SetVec3("light_vec", light_vec);
  rope_shader.SetMat4("view_proj_mat", view_projection);
  for (size_t i = 0; i + 1 < rope.verts.size(); ++i) {
    dvec3 a = rope.verts[i].pos, b = rope.verts[i+1].pos;
    rope_shader.SetMat4("model_mat",
                        fmat4::Translation((a+b)/2) *
                        fquat::RotationTo(fvec3(0, 1, 0), b-a).ToMatrix4() *
                        fmat4::Scale(rope.radius, (b-a).Length()/2, rope.radius));
    rope_segment_mesh->Draw();
  }
}
