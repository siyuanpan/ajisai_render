import os
import sys
import subprocess
import argparse
import shutil

def build(srcdir, builddir, target=[], debug=False, toolchain=None, makeprogram='Ninja', cflags=[]):
    cmake_args = ['-DCMAKE_BUILD_TYPE=' + ('Debug' if debug else 'Release')]
    if not toolchain is None:
        cmake_args += ['-DCMAKE_TOOLCHAIN_FILE=' + toolchain]
    if makeprogram == 'Ninja':
        cmake_args += ['-G', 'Ninja', '-DCMAKE_MAKE_PROGRAM=ninja']

    os.makedirs(builddir, exist_ok=True)

    subprocess.check_call(['cmake', '-S', srcdir, '-B', builddir] + cmake_args, cwd=srcdir)
    subprocess.check_call(['cmake', '--build', builddir, '--', '-j'], cwd=srcdir)
    

ap = argparse.ArgumentParser()
ap.add_argument("-d", "--debug", action='store_true', help='build with debug')
ap.add_argument("-b", "--build", action='store_true', default=True, help='build project')
ap.add_argument("-p", "--with_parallel", action='store_true', default=True, help='build with parallel')
ap.add_argument("-c", "--clean", action='store_true', help='clean')
ap.add_argument("-r", "--run", action='store_true', help='run app')

args = vars(ap.parse_args())

# build
srcdir = os.getcwd() # os.path.join(os.getcwd(), "src")
builddir = os.path.join(os.getcwd(), "build_release" if not args['debug'] else "build_debug")

if not args['with_parallel']:
    builddir += '_np'

if args['clean']:
    shutil.rmtree(builddir)

toolchain = "./vcpkg/scripts/buildsystems/vcpkg.cmake"

if args['build']:
    build(srcdir=srcdir, builddir=builddir, debug=args['debug'], toolchain=toolchain, makeprogram='GNU Makefile')

if args['run']:
    subprocess.run([os.path.join(builddir, 'Ajisai')], cwd=os.getcwd())