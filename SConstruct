import os

env = SConscript("godot-cpp/SConstruct")

env.Append(CXXFLAGS=["-std=c++20", "-O3", "-fexceptions"])
env.Append(CCFLAGS=["-O3", "-fexceptions"])
env.Append(CPPPATH=["src/"])

sources = Glob("src/*.cpp")

# إخراج المكتبة مباشرة في مسار مشروع جودو
library = env.SharedLibrary(
    "project/bin/android/libcarstudio{}{}".format(env["suffix"], env["SHLIBSUFFIX"]),
    source=sources,
)

Default(library)
