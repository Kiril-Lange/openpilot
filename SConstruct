import os
import subprocess

zmq = 'zmq'
arch = subprocess.check_output(["uname", "-m"], encoding='utf8').rstrip()

cereal_dir = Dir('.')

cpppath = [
<<<<<<< HEAD
    cereal_dir,
=======
    '#',
    '#cereal',
    "#cereal/messaging",
    "#opendbc/can",
>>>>>>> 683b6151... Squashed 'opendbc/' changes from c0eba096..4f82d01e
    '/usr/lib/include',
]

AddOption('--test',
          action='store_true',
          help='build test files')

AddOption('--asan',
          action='store_true',
          help='turn on ASAN')

ccflags_asan = ["-fsanitize=address", "-fno-omit-frame-pointer"] if GetOption('asan') else []
ldflags_asan = ["-fsanitize=address"] if GetOption('asan') else []

env = Environment(
  ENV=os.environ,
  CC='clang',
  CXX='clang++',
  CCFLAGS=[
    "-g",
    "-fPIC",
    "-O2",
    "-Werror=implicit-function-declaration",
    "-Werror=incompatible-pointer-types",
    "-Werror=int-conversion",
    "-Werror=return-type",
    "-Werror=format-extra-args",
  ] + ccflags_asan,
  LDFLAGS=ldflags_asan,
  LINKFLAGS=ldflags_asan,

  CFLAGS="-std=gnu11",
  CXXFLAGS="-std=c++14",
  CPPPATH=cpppath,
)

<<<<<<< HEAD

Export('env', 'zmq', 'arch')
SConscript(['SConscript'])
=======
Export('env', 'zmq', 'arch')

cereal = [File('#cereal/libcereal.a')]
messaging = [File('#cereal/libmessaging.a')]
Export('cereal', 'messaging')

SConscript(['cereal/SConscript'])
SConscript(['opendbc/can/SConscript'])
>>>>>>> 683b6151... Squashed 'opendbc/' changes from c0eba096..4f82d01e
