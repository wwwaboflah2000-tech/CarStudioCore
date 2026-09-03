import os

env = SConscript("godot-cpp/SConstruct", {"api_version": "4.7"})

env.Append(CXXFLAGS=["-std=c++20", "-O3", "-fexceptions"])
env.Append(CCFLAGS=["-O3", "-fexceptions"])
env.Append(CPPPATH=["#src/", "src/", "#"])

sources = Glob("src/*.cpp")

library = env.SharedLibrary(
    "bin/android/libcarstudio{}{}".format(env["suffix"], env["SHLIBSUFFIX"]),
    source=sources,
)

Default(library)
