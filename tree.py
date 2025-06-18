import os
import fnmatch
import re

IGNORE_PATTERNS = [
    # Folder patterns
    "build", "solvers", "[Ll]ibrary", "[Tt]emp", "[Oo]bj", "[Bb]uild", "[Bb]uilds",
    "[Ll]ogs", "[Mm]emoryCaptures", "UserSettings", ".idea", ".vscode", "__pycache__",
    ".vs", ".gradle", ".git", ".svn", ".hg", ".cproject", ".project", ".settings",
    ".classpath", ".DS_Store", ".vscode-test", "node_modules", "bower_components",
    "dist", "out", "coverage", "docs", "doc", "test", "tests", "examples",
    "example", "samples", "sample", "vendor", "lib", "libs", "assets", "cache",
    "temp", "tmp", "logs", "log", "output", "outputs", "builds", "bin", "obj",
    "debug", "release", "releases", "staging", "production", "deploy", "deployment",
    "generated", "gen", "builds", "artifacts", "releases", "releases-*", "environment",
    "Scenes", "Settings", "TutorialInfo", "Tutorials", "TutorialsInfo", "TutorialsData",
    

    # File patterns
    "*.csproj", "*.unityproj", "*.sln", "*.suo", "*.tmp", "*.user", "*.userprefs",
    "*.pidb", "*.booproj", "*.svd", "*.pdb", "*.mdb", "*.opendb", "*.VC.db",
    "*.pidb.meta", "*.bak", "*.swp", "*.lock", "*.DS_Store", "*.apk", "*.aab",
    "*.unitypackage", "*.orig", "crashlytics-build.properties", "*.meta", "*.asset",
    "*.dll", "*.exe", "*.so", "*.dylib", "*.lib", "*.a", "*.o", "*.obj",
    "*.inputactions", "*.json", "*.xml", "*.yaml", "*.yml", "*.txt",
    "*.md", "*.markdown", "*.html", "*.htm", "*.css", "*.js", "*.ts",
]

def should_ignore(name):
    return any(fnmatch.fnmatch(name, pattern) for pattern in IGNORE_PATTERNS)

def print_tree(start_path, prefix=""):
    try:
        entries = sorted(os.listdir(start_path))
    except PermissionError:
        return  # skip unreadable dirs

    entries = [e for e in entries if not should_ignore(e)]

    for i, entry in enumerate(entries):
        path = os.path.join(start_path, entry)
        connector = "└── " if i == len(entries) - 1 else "├── "
        print(prefix + connector + entry)
        if os.path.isdir(path):
            extension = "    " if i == len(entries) - 1 else "│   "
            print_tree(path, prefix + extension)

print_tree("/home/eren/GitHub/ControlSystem")
