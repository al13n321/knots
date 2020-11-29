#include <vector>
#include <unordered_map>

// Sparse matrix, interpreted as system of linear equations.
struct SparseMatrix {
  // Initially these are two representations of the same data: the set of nonzero elements.
  // During Gaussian elimination they take on a different meaning:
  // `rows` contains the full current state of the matrix as usual, while
  // `columns` contains only elements whose row and column haven't been eliminated yet.
  std::vector<std::unordered_map<int, double>> rows;
  std::vector<std::unordered_map<int, double>> columns;

  void Init(size_t nr, size_t nc) {
    rows.assign(nr, {});
    columns.assign(nc, {});
  }

  void Add(int r, int c, double v) {
    rows.at(r)[c] += v;
    columns.at(c)[r] += v;
  }

  // Does a step of Gaussian elimination: eliminates a given equation and a given variable.
  // If the element at [r, c] is zero, returns false and does nothing.
  bool Eliminate(int r, int c, double eps) {
    // Get M[r][c].
    auto& row = rows.at(r);
    double v;
    {
      auto it = row.find(c);
      if (it == row.end()) return false;
      v = it->second;
    }
    if (fabs(v) <= eps) return false;
    double iv = 1/v;

    // Do the elimination.
    for (auto p : columns.at(c)) {
      if (p.first == r) continue;
      double coef = -p.second * iv;
      auto& row2 = rows.at(p.first);
      size_t rv = row2.erase(c);
      assert(rv);
      for (auto q : row) {
        if (q.first == c) continue;
        double x = (row2[q.first] += coef * q.second);
        columns[q.first][p.first] = x;
      }
    }

    // Remove the eliminated column from `columns`.
    columns[c].clear();
    
    // Remove the eliminated row from `columns`.
    for (auto p : row) {
      auto& col = columns.at(p.first);
      auto it = col.find(r);
      if (p.first == c) {
        assert(it == col.end());
        continue;
      }
      assert(it != col.end());
      col.erase(it);
    }
    
    return true;
  }

  // Does back-substitution of eliminated variables into eliminated equations.
  // The matrix M is interpreted as system M*x=0, where the last element of `x` is a fake extra variable equal to 1.
  // `order` must contain [r, c] pairs for successful eliminations, in order of elimination.
  // If system has no solutions, i.e. some equations can't be satisfied (Eliminate() returned false for them), produces a solution
  // for the rest of equations and populates  out_lhs with left-hand sides of all equations (note that we don't reorder and don't multiply rows).
  // The order of eliminations effectively decides the priority of equations: earlier Eliminate() calls are more likely to succeed;
  // e.g. maybe you can process constraints that hold an object together first, and constraints corresponding to user carrying an object last,
  // and provide feedback to the user if their constraint couldn't be satisfied.
  // If system has multiple solutions, i.e. some variables didn't participate in successful eliminations and don't occur in `order`,
  // these variables are set to zero; similarly to equations, variables whose Eliminate() call occurred later
  // are more likely to be chosen as free variables and be set to zero; not sure if that's relevant in practice.
  void BackSubstitute(const std::vector<std::pair<int, int>>& order,
                      std::vector<double>& out_vars,
                      std::vector<double>& out_lhs) {
    out_vars.assign(columns.size(), 0);
    out_vars.back() = 1;
    out_lhs.assign(rows.size(), 0);
    std::vector<bool> equations_seen(rows.size());
    for (int i = (int)order.size() - 1; i >= 0; --i) {
      int r = order[i].first;
      assert(!equations_seen.at(r));
      equations_seen[r] = true;
      int c = order[i].second;
      auto& row = rows.at(r);
      double s = 0;
      for (auto p : row) {
        if (p.first == c) continue;
        s += p.second * out_vars.at(p.first);
      }
      assert(row.count(c));
      double v = row.at(c);
      assert(v != 0);
      out_vars[c] = -s/row.at(c);
    }
    for (int r = 0; r < rows.size(); ++r) {
      if (equations_seen[r]) continue;
      double s = 0;
      for (auto p : rows.at(r)) s += p.second * out_vars.at(p.first);
      out_lhs.at(r) = s;
    }
  }
};
