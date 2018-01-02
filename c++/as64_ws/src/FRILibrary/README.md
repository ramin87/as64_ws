FRILibrary
================================
A custom version of [Fast Research Interface Library](http://cs.stanford.edu/people/tkr/fri/html/)

Dependecies
-------------------------
* Ubuntu 14.04
* [gcc-multilib](http://packages.ubuntu.com/trusty/gcc-multilib)
* [g++-multilib](http://packages.ubuntu.com/trusty/g++-multilib)

Manual Build
-------------------------
1.  Install gcc-multilib && g++-multilib

    ```bash
    sudo apt-get install -y gcc-multilib g++-multilib
    ```
2. Build library

    ```bash
    cd Linux
    make clean all
    ```
3. Add FRIL_PATH (repository root) to your ~/.bashrc


Automatic Build
-------------------------
Run build script from root folder

```bash
./build.sh
```
Important
-------------------------
Running both the manual and automatic instructions may result in duplicate FRIL_PATH entry.