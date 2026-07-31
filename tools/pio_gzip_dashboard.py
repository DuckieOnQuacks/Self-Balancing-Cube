# PlatformIO pre: hook. Runs inside SCons, where __file__ is not defined, so
# the project root comes from the build environment. The real generator is a
# plain script so it can also be run by hand: python tools/gzip_dashboard.py
import os
import runpy

Import("env")  # noqa: F821  (SCons injects this)

runpy.run_path(os.path.join(env["PROJECT_DIR"], "tools", "gzip_dashboard.py"))  # noqa: F821
