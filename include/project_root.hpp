/*
░█▀█░█▀▄░█▀█░▀▀█░█▀▀░█▀▀░▀█▀░░░░░█▀▄░█▀█░█▀█░▀█▀
░█▀▀░█▀▄░█░█░░░█░█▀▀░█░░░░█░░░░░░█▀▄░█░█░█░█░░█░
░▀░░░▀░▀░▀▀▀░▀▀░░▀▀▀░▀▀▀░░▀░░▀▀▀░▀░▀░▀▀▀░▀▀▀░░▀░

project_root.hpp - Locates the repository root at runtime (CLI argument, working directory or executable path).
*/

#pragma once

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
