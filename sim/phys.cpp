#pragma warning(disable:4267) // implicit conversions between integer types
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
    verts[i].mass = density * max_segment_length * ((i==0 || i==n) ? .5 : 1.);
    if (i) verts[i-1].target_length = (verts[i].pos - verts[i-1].pos).Length();
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
  nvars += scene.external_constraints.size() * 3;
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
    double im = 1/verts[c.vert_idx].mass;
    // Imulse applied directly to the vert.
    ctx.verts[c.vert_idx].vars.emplace_back(ctx.external_constraint_vars_idx + i*3 + 0, dvec3(im, 0, 0));
    ctx.verts[c.vert_idx].vars.emplace_back(ctx.external_constraint_vars_idx + i*3 + 1, dvec3(0, im, 0));
    ctx.verts[c.vert_idx].vars.emplace_back(ctx.external_constraint_vars_idx + i*3 + 2, dvec3(0, 0, im));
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
    // Equation that vert velocity is equal to the one in constraint.
    ctx.equations.Add(ctx.external_constraint_vars_idx + i*3 + 0, ctx.constant_term_idx, -c.vel.x);
    ctx.equations.Add(ctx.external_constraint_vars_idx + i*3 + 1, ctx.constant_term_idx, -c.vel.y);
    ctx.equations.Add(ctx.external_constraint_vars_idx + i*3 + 2, ctx.constant_term_idx, -c.vel.z);
    for (const auto& [idx, v] : ctx.verts.at(c.vert_idx).vars) {
      ctx.equations.Add(ctx.external_constraint_vars_idx + i*3 + 0, idx, v.x);
      ctx.equations.Add(ctx.external_constraint_vars_idx + i*3 + 1, idx, v.y);
      ctx.equations.Add(ctx.external_constraint_vars_idx + i*3 + 2, idx, v.z);
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

  // Calculate masses of verts.
  for (auto& v : rope.verts) v.mass = 0;
  double mass_per_unit_length = rope.density * M_PI * pow(rope.radius, 2.);
  for (size_t i = 0; i + 1 < rope.verts.size(); ++i) {
    double d = rope.verts[i].target_length;
    rope.verts[i  ].mass += d/2 * mass_per_unit_length;
    rope.verts[i+1].mass += d/2 * mass_per_unit_length;
  }

  // Apply gravity.
  for (auto& v : rope.verts) v.vel += gravity * dt;

  // todo: more forces (gravity, straightening, drag, angular kinetic friction)
  // todo: iterate on the set of normal reaction forces

  ResolveLinearConstraints(*this, ctx);

  // todo: apply friction and solve constraints again (iterating again, but with good initial guess)

  // Report unmet constraints.
  for (size_t i = 0; i < external_constraints.size(); ++i) {
    bool ok = true;
    for (size_t j = 0; j < 3; ++j) {
      ok &= fabs(ctx.equations_residue.at(ctx.external_constraint_vars_idx + i*3 + j)) <= 1e-6;
    }
    external_constraints.at(i).satisfied = ok;
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
