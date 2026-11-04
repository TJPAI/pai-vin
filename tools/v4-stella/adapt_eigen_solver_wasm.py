#!/usr/bin/env python3
"""Strict, pinned Stella WASM adaptation: use genuine g2o Eigen solvers, not fake CSparse targets.

This is a build experiment, not evidence of numerical equivalence or successful SLAM.
Run against upstream stella_vslam commit e445b5452535e781fdf6777ec33a06f5f8f5e416.
"""
from pathlib import Path
import argparse

EXPECTED = {
    "src/stella_vslam/optimize/global_bundle_adjuster.cc": [
        ("#include <g2o/solvers/csparse/linear_solver_csparse.h>\n", ""),
        ("g2o::LinearSolverCSparse<g2o::BlockSolver_6_3::PoseMatrixType>",
         "g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType>"),
    ],
    "src/stella_vslam/optimize/graph_optimizer.cc": [
        ("#include <g2o/solvers/csparse/linear_solver_csparse.h>",
         "#include <g2o/solvers/eigen/linear_solver_eigen.h>"),
        ("g2o::LinearSolverCSparse<g2o::BlockSolver_7_3::PoseMatrixType>",
         "g2o::LinearSolverEigen<g2o::BlockSolver_7_3::PoseMatrixType>"),
    ],
    "src/stella_vslam/CMakeLists.txt": [
        ("             g2o::solver_csparse\\n             g2o::csparse_extension\\n             OPTIONAL_COMPONENTS\\n             g2o::csparse", ""),
        ("                      g2o::solver_csparse\\n", ""),
        ("                      g2o::csparse_extension\\n", ""),
    ],
}

def apply(root: Path, check: bool = False) -> None:
    planned = {}
    for rel, replacements in EXPECTED.items():
        path = root / rel
        source = path.read_text()
        updated = source
        for old, new in replacements:
            count = updated.count(old)
            if count != 1:
                raise RuntimeError(f"{rel}: expected exactly 1 occurrence of {old!r}; found {count}")
            updated = updated.replace(old, new, 1)
        planned[path] = updated
    for path, updated in planned.items():
        if not check:
            path.write_text(updated)
        print(f"{'Verified' if check else 'Patched'}: {path.relative_to(root)}")
    print("Stella Eigen solver WASM adaptation validated. Numerical/SLAM results NOT verified.")

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("upstream_root", type=Path)
    parser.add_argument("--check", action="store_true")
    args = parser.parse_args()
    apply(args.upstream_root.resolve(), args.check)
