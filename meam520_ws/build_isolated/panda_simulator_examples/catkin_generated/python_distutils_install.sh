#!/bin/sh

if [ -n "$DESTDIR" ] ; then
    case $DESTDIR in
        /*) # ok
            ;;
        *)
            /bin/echo "DESTDIR argument must be absolute... "
            /bin/echo "otherwise python's distutils will bork things."
            exit 1
    esac
fi

echo_and_run() { echo "+ $@" ; "$@" ; }

echo_and_run cd "/home/student/meam520_ws/src/panda_simulator/panda_simulator_examples"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/student/meam520_ws/install_isolated/lib/python3/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/student/meam520_ws/install_isolated/lib/python3/dist-packages:/home/student/meam520_ws/build_isolated/panda_simulator_examples/lib/python3/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/student/meam520_ws/build_isolated/panda_simulator_examples" \
    "/usr/bin/python3" \
    "/home/student/meam520_ws/src/panda_simulator/panda_simulator_examples/setup.py" \
     \
    build --build-base "/home/student/meam520_ws/build_isolated/panda_simulator_examples" \
    install \
    --root="${DESTDIR-/}" \
    --install-layout=deb --prefix="/home/student/meam520_ws/install_isolated" --install-scripts="/home/student/meam520_ws/install_isolated/bin"
