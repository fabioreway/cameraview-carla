This is a simple application to create and spawn virtual cameras in a vehicle in CARLA once simulation is up and running.

It was implemented to investigate the performance of the CARLA simulation when multiple cameras are created.

The results are published and discussed here:
https://github.com/carla-simulator/carla/issues/3836

To run this app, place it in the .../CARLA/Examples/ folder and compile it:

```console
reway@home:~$ make
```

To display help:
```console
reway@home:~$ ./bin/cpp_client --help
A simple code to deploy virtual cameras in a CARLA actor
Usage:
  CARLA CameraView [OPTION...]

  -s, --server arg       CARLA Server (default: localhost)
  -p, --port arg         Port number (default: 2000)
  -i, --actor-id arg     Actor ID (default: 86)
  -n, --num-cams arg     Number of cameras (default: 4)
  -m, --max-cams         Deploy maximum number of cameras
  -x, --resx arg         Resolution in X (default: 1920)
  -y, --resy arg         Resolution in Y (default: 1232)
  -f, --fieldofview arg  Camera Field of View (default: 60)
  -h, --help             Print usage

reway@home:~/05_simulators/CARLA-cpp/CARLA/Examples/CameraView$ ./bin/cpp_client ....
```


