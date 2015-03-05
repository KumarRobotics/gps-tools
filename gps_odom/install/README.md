### Installing GeographicLib

1. Download the latest version of GeographicLib from [SourceForge](http://sourceforge.net/projects/geographiclib/files/distrib/).

2. Build and install GeographicLib using the following commands (assuming you are in the `GeographicLib-1.35` directory):
  ```bash
  mkdir build
  cd build
  cmake ..
  make -j4
  sudo make install
  ```

### For version 1.35-36

Versions 1.35 and 1.36 have a makefile failure which can be corrected using the patch provided in this folder:

1. `cd` into the directory you downloaded, and apply `fixgeo.patch` using the following command:
  ```bash
  patch -p1 < fixgeo.patch
  ```
2. You should see output indicating that a documentation file has been patched.

3. Build GeoGraphiclib and install it, as specified above. **gps_odom** should now build without issue.
