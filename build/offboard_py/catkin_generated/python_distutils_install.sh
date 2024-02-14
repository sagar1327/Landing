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

echo_and_run cd "/home/sagar/ROS/src/offboard_py"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/sagar/ROS/install/lib/python3/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/sagar/ROS/install/lib/python3/dist-packages:/home/sagar/ROS/build/lib/python3/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/sagar/ROS/build" \
    "/usr/bin/python3" \
    "/home/sagar/ROS/src/offboard_py/setup.py" \
     \
    build --build-base "/home/sagar/ROS/build/offboard_py" \
    install \
    --root="${DESTDIR-/}" \
    --install-layout=deb --prefix="/home/sagar/ROS/install" --install-scripts="/home/sagar/ROS/install/bin"
