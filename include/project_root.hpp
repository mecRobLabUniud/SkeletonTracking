/*
░PROJECT_ROOT░
░PROJECT_ROOT░
░PROJECT_ROOT░

project_root.hpp
*/

#pragma once

/*
░█▀▄░█▀█░▀█▀░█▀█░░░▀█▀░█▀▄░█▀█░█▀█░█▀▀░█▄█░▀█▀░▀█▀░▀█▀░█▀▀░█▀▄
░█░█░█▀█░░█░░█▀█░░░░█░░█▀▄░█▀█░█░█░▀▀█░█░█░░█░░░█░░░█░░█▀▀░█▀▄
░▀▀░░▀░▀░░▀░░▀░▀░░░░▀░░▀░▀░▀░▀░▀▀▀░▀▀▀░▀▀▀░▀▀▀░░▀▀▀▀▀░▀░▀

Location of the repository root, i.e. of the directory holding src/urdf/,
src/trajectories/ and scripts/.

The root is looked up in this order:
  1) the path given on the command line (run.sh passes the repository root)
  2) the current working directory
  3) the parent of the executable's directory (build/ -> project root)

so that the executables can be launched either from the build directory or
from the project root, with no path depending on the current working
directory.
*/

#include <filesystem>
#include <string>

// ─────────────────────────────────────────────────────────────────────────────
// Locate the project root, i.e. the directory holding src/urdf/panda.urdf
// ─────────────────────────────────────────────────────────────────────────────
inline std::filesystem::path findProjectRoot(const std::string& explicit_root = "") {
    namespace fs = std::filesystem;
    std::error_code ec;

    const auto is_root = [](const fs::path& p) {
        std::error_code ec2;
        return fs::exists(p / "src" / "urdf" / "panda.urdf", ec2);
    };

    // 1) the path given on the command line
    if (!explicit_root.empty()) {
        fs::path p = fs::path(explicit_root);
        if (p.is_relative()) p = fs::absolute(p, ec);
        if (!ec && is_root(p)) return p;
    }

    // 2) the current working directory
    fs::path cwd = fs::current_path(ec);
    if (!ec && is_root(cwd)) return cwd;

    // 3) the parent of the executable's directory (build/ -> project root)
    fs::path exe = fs::read_symlink("/proc/self/exe", ec);
    if (!ec) {
        fs::path root = exe.parent_path().parent_path();
        if (is_root(root)) return root;

        root = exe.parent_path();
        if (is_root(root)) return root;
    }

    return cwd;  // fallback: the files below will simply fail to open
}
