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
    DESTDIR_ARG="--root=$DESTDIR"
fi

echo_and_run() { echo "+ $@" ; "$@" ; }

echo_and_run cd "/home/kartik/Documents/gazebo_practice_ws/src/autonomous_vehicle/catvehicle_basics/catvehicle"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/kartik/Documents/gazebo_practice_ws/install/lib/python2.7/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/kartik/Documents/gazebo_practice_ws/install/lib/python2.7/dist-packages:/home/kartik/Documents/gazebo_practice_ws/build/lib/python2.7/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/kartik/Documents/gazebo_practice_ws/build" \
    "/usr/bin/python2" \
    "/home/kartik/Documents/gazebo_practice_ws/src/autonomous_vehicle/catvehicle_basics/catvehicle/setup.py" \
    build --build-base "/home/kartik/Documents/gazebo_practice_ws/build/autonomous_vehicle/catvehicle_basics/catvehicle" \
    install \
    $DESTDIR_ARG \
    --install-layout=deb --prefix="/home/kartik/Documents/gazebo_practice_ws/install" --install-scripts="/home/kartik/Documents/gazebo_practice_ws/install/bin"
