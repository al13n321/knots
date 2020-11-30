#ifdef _WIN32
#pragma warning(disable:4267) // implicit conversions between integer types
#endif
#include "sim/scene.h"
#include "util/linear.h"
#include "util/print.h"
#include "util/sparse.h"
#include <valarray>
#include <cassert>
#include <iostream>
using namespace std;

namespace {

struct VertInfo {
  // Vert velocity as linear combination of variables.
  std::vector<std::pair<int, dvec3>> vars;
  // Tentative velocity change to apply to this vert.
  dvec3 dv;
};

struct Context {
  // Beginnings of ranges of variable indices (n is number of segments, m is number of contacts).
  // n segment length preservation impulses (pointed inwards, can be negative).
  size_t segment_length_vars_idx = 0;
  // Impulses for satisfying external_constraints.
  size_t external_constraint_vars_idx;
  // One pseudovariable always equal to 1. Needs to be last because SparseMatrix expects it.
  size_t constant_term_idx;
  //   todo: m collision avoidance impulses (pointed away from collision, should be nonnegative)
  //   todo: 2m friction impulses (2 vars per contact), unused if kinetic friction

  std::vector<VertInfo> verts;
  SparseMatrix equations;

  // Indices of Gaussian-eliminated equations and variables, in order of elimination.
  // These vars are removed from equations.columns (so future eliminations don't affect them),
  // but remain in equations.rows (holding the coefficients that we'll need for the back-substitution).
  std::vector<std::pair<int, int>> eliminations;

  // Solution to `equations`.
  std::vector<double> solution;
  // Left-hand side of each equation after substituting `solution` for variable values.
  // Elements whose absolute value is above epsilon correspond to constraints that couldn't be met.
  std::vector<double> equations_residue;
};

}

void Scene::PrintStats() const {
  double e = 0;
  for (const auto& v : rope.verts) e += v.mass * (v.vel.LengthSquare() / 2 - gravity.Dot(v.pos));
  double l = 0;
  for (size_t i = 0; i + 1 < rope.verts.size(); ++i) l += (rope.verts[i+1].pos - rope.verts[i].pos).Length();
  std::cerr << "energy: " << e << ", length: " << l << std::endl;
}

void Rope::Create(size_t n) {
  verts.resize(n+1);
  // arrange into a semicircle for now
  double r = n*max_segment_length/M_PI;
  for(size_t i = 0; i <= n; ++i) {
    double ang = M_PI*i/n;
    verts[i].pos = dvec3(r*sin(ang), -r+r*cos(ang), 0);
    verts[i].vel = dvec3();
    if (i) verts[i-1].target_length = (verts[i].pos - verts[i-1].pos).Length();
  }
  RecalculateMasses();
}

void Rope::RecalculateMasses() {
  // Calculate masses of verts.
  for (auto& v : verts) v.mass = 0;
  double mass_per_unit_length = density * M_PI * radius * radius;
  for (size_t i = 0; i + 1 < verts.size(); ++i) {
    double d = verts[i].target_length;
    verts[i  ].mass += d/2 * mass_per_unit_length;
    verts[i+1].mass += d/2 * mass_per_unit_length;
  }
}

// Fill out VertInfo-s and equations, and solve the equations.
// Doesn't update actual vert velocities, and doesn't care about variable sign or magnitude constraints.
static void ResolveLinearConstraints(Scene& scene, Context& ctx) {
  const auto& verts = scene.rope.verts;
  size_t nvars = 0;
  ctx.segment_length_vars_idx = nvars;
  nvars += verts.size() - 1;
  ctx.external_constraint_vars_idx = nvars;
  nvars += scene.external_constraints.size();
  ctx.constant_term_idx = nvars;
  ctx.verts.assign(verts.size(), {});

  // Fill out VertInfo.
  // Initial velocity (constant term).
  for (size_t i = 0; i < verts.size(); ++i) {
    ctx.verts[i].vars.emplace_back(ctx.constant_term_idx, verts[i].vel);
  }
  // Length preservation impulse.
  for (size_t i = 0; i + 1 < verts.size(); ++i) {
    dvec3 d = (verts[i+1].pos - verts[i].pos).Normalized();
    ctx.verts[i  ].vars.emplace_back(ctx.segment_length_vars_idx + i,  d/verts[i  ].mass);
    ctx.verts[i+1].vars.emplace_back(ctx.segment_length_vars_idx + i, -d/verts[i+1].mass);
  }
  // External constrains impulses.
  for (size_t i = 0; i < scene.external_constraints.size(); ++i) {
    const auto& c = scene.external_constraints[i];
    // Imulse applied directly to the vert.
    ctx.verts[c.vert_idx].vars.emplace_back(ctx.external_constraint_vars_idx + i, c.axis/verts[c.vert_idx].mass);
  }

  // Fill out equations.
  ctx.equations.Init(nvars, nvars + 1);
  // Length preservation.
  for (size_t i = 0; i + 1 < verts.size(); ++i) {
    // The equation is (v1-v2).Dot(d) = contraction_speed
    dvec3 d0 = verts[i+1].pos - verts[i].pos;
    double l = d0.Length();
    dvec3 d = d0/l;
    for (const auto& [idx, v1] : ctx.verts[i  ].vars) ctx.equations.Add(ctx.segment_length_vars_idx + i, idx,  v1.Dot(d));
    for (const auto& [idx, v2] : ctx.verts[i+1].vars) ctx.equations.Add(ctx.segment_length_vars_idx + i, idx, -v2.Dot(d));
    // Integration error tends to make segments longer, sometimes very quickly. Here we fudge the length-preservation
    // equation to require length to change towards the target length instead of staying constant.
    double contraction_rate = verts[i].target_length * (l/verts[i].target_length - 1) * scene.segment_length_restoration_fudge_factor;
    ctx.equations.Add(ctx.segment_length_vars_idx + i, ctx.constant_term_idx, -contraction_rate);
  }
  // External constraints.
  for (size_t i = 0; i < scene.external_constraints.size(); ++i) {
    const auto& c = scene.external_constraints[i];
    // Equation that vert's speed along constraint's axis is equal to the speed requested in the constraint.
    ctx.equations.Add(ctx.external_constraint_vars_idx + i, ctx.constant_term_idx, -c.speed);
    for (const auto& [idx, v] : ctx.verts.at(c.vert_idx).vars) {
      double proj = c.axis.Dot(v);
      if (abs(proj) > 1e-12) ctx.equations.Add(ctx.external_constraint_vars_idx + i, idx, proj);
    }
  }

  // Solve the equations.
  const double eps = 1e-9;
  auto original_equations = ctx.equations; // todo: remove
  ctx.eliminations.clear();
  for (int i = 0; i < nvars; ++i) {
    if (ctx.equations.Eliminate(i, i, eps)) ctx.eliminations.emplace_back(i, i);
  }
  ctx.equations.BackSubstitute(ctx.eliminations, ctx.solution, ctx.equations_residue);
  int unsatisfied = 0;
  for (double v : ctx.equations_residue) unsatisfied += fabs(v) > 1e-9;
  if (unsatisfied != 0) std::cerr << unsatisfied << " unsatisfied equations" << std::endl;
  for (int r = 0; r < original_equations.rows.size(); ++r) {
    double s = 0;
    for (auto p : original_equations.rows.at(r)) s += p.second * ctx.solution.at(p.first);
    assert(fabs(s - ctx.equations_residue.at(r)) <= 1e-9);
  }
}

static void GetVertsDv(Scene& scene, Context& ctx) {
  for (size_t i = 0; i < ctx.verts.size(); ++i) {
    auto& vert = ctx.verts[i];
    vert.dv = {};
    for (const auto& p : vert.vars) if (p.first != ctx.constant_term_idx) vert.dv += p.second * ctx.solution.at(p.first);
  }
}

void Scene::PhysicsStep(double dt) {
  Context ctx;

  // Apply various external forces.

  // Gravity.
  for (auto& v : rope.verts) v.vel += gravity * dt;
  // Drag and lift.
  for (size_t i = 0; i + 1 < rope.verts.size(); ++i) {
    dvec3 d = rope.verts[i].pos - rope.verts[i+1].pos;
    // We're ignoring rotation and pretend that the segment is just moving linearly, at the velocity of its center.
    // That should be ok because segments are short, so their rotation doesn't produce strong forces.
    dvec3 vel = (rope.verts[i].vel + rope.verts[i+1].vel)/2;
    double speed_squared = vel.LengthSquare();
    double speed = sqrt(speed_squared);
    double length = d.Length();

    if (speed < 1e-3 || length < 1e-5) continue;

    // Simplified model of drag and lift for a cylinder, ignoring Reynolds number, torque, interaction with adjacent segments, etc.
    // No idea if that's a reasonable approximation of what happens between a rope and air.

    // Angle of attack normalized to [0, 1] (0 is 0 degrees, 1 is 90 degrees).
    double aoa = acos(vel.Dot(d)/speed/length)/M_PI*2;
    aoa = std::min(aoa, 2 - aoa);
    // Drag coefficient as function of angle of attack. Made up after staring a bit at graphs in papers
    // "Drag and lift coefficients of inclined finite circular cylinders at moderate Reynolds numbers" and
    // "A method for improving the dynamic simulation efficiency of underwater flexible structures by implementing non-active points in modelling".
    // Normalized by cylinder's length*diameter, not by projected area.
    double drag_amount = aoa*aoa*(3-aoa*2); // smoothstep, going from 0 to 1, with zero derivative at both ends (which makes physical sense because of symmetry)
    // Lift correction coefficient, similarly pulled out of thin air based on graphs in the above two papers.
    // Note that below we're multiplyint this by sin(angle of attack).
    double lift_amount = (1-aoa)*2*1.414;

    dvec3 drag = -vel * (speed * length * rope.radius * air_density * drag_amount * rope.drag_coefficient);

    // Perpendicular to velocity, coplanar with velocity and the segment,
    // length equal to sin(angle of attack) * segment_length * speed_squared.
    dvec3 lift = d.Cross(vel).Cross(vel) * (d.Dot(vel) > 0 ? -1 : 1);
    lift *= air_density * rope.radius * lift_amount * rope.lift_coefficient;

    dvec3 force = lift + drag;
    
    rope.verts[i  ].vel += force * (dt / rope.verts[i  ].mass / 2);
    rope.verts[i+1].vel += force * (dt / rope.verts[i+1].mass / 2);
  }
  // Elasticity of the joints (aka straightening force).
  for (size_t i = 1; i + 1 < rope.verts.size(); ++i) {
    dvec3 v1 = rope.verts[i-1].pos - rope.verts[i].pos;
    dvec3 v2 = rope.verts[i+1].pos - rope.verts[i].pos;
    double l1 = v1.Length(), l2 = v2.Length();
    double l = (l1 + l2) / 2;

    double torsion_coefficient = rope.modulus_of_elasticity * pow(rope.radius, 4) / l;
    // Length is sin(ang) * l1 * l2. We'll assume that sin(ang) is close enough to ang.
    dvec3 axis = v1.Cross(v2);
    
    dvec3 force1 = axis.Cross(v1) * (torsion_coefficient / l1 / l2);
    rope.verts[i-1].vel -= force1 * (dt / rope.verts[i-1].mass);
    rope.verts[i  ].vel += force1 * (dt / rope.verts[i  ].mass);

    dvec3 force2 = axis.Cross(v2) * (torsion_coefficient / l1 / l2);
    rope.verts[i+1].vel += force2 * (dt / rope.verts[i+1].mass);
    rope.verts[i  ].vel -= force2 * (dt / rope.verts[i  ].mass);
  }

  // todo: friction
  // todo: torsion

  // Resolve constraints.

  // todo: iterate on the set of normal reaction forces

  ResolveLinearConstraints(*this, ctx);

  // todo: apply friction and solve constraints again (iterating again, but with good initial guess)

  // Report unmet constraints.
  for (size_t i = 0; i < external_constraints.size(); ++i) {
    external_constraints.at(i).satisfied = fabs(ctx.equations_residue.at(ctx.external_constraint_vars_idx + i)) <= 1e-6;
  }

  // Apply velocity updates and advance positions.
  GetVertsDv(*this, ctx);
  for (size_t i = 0; i < rope.verts.size(); ++i) {
    rope.verts[i].vel += ctx.verts[i].dv;
    rope.verts[i].pos += rope.verts[i].vel * dt;
  }

  // todo: resegment
  // todo: detect collisions
  // todo: fudge positions (per component, allow length change)
}
