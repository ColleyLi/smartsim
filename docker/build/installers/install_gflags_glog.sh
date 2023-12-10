set -e

cd "$(dirname "${BASH_SOURCE[0]}")"
. ./installer_base.sh

ARCH=$(uname -m)
THREAD_NUM=$(nproc)

# Install gflags.
VERSION="2.2.2"
CHECKSUM="34af2f15cf7367513b352bdcd2493ab14ce43692d2dcd9dfc499492966c64dcf"
PKG_NAME="gflags-${VERSION}.tar.gz"
DOWNLOAD_LINK="https://github.com/gflags/gflags/archive/v${VERSION}.tar.gz"

download_if_not_cached "${PKG_NAME}" "${CHECKSUM}" "${DOWNLOAD_LINK}"

tar xzf "${PKG_NAME}"

pushd gflags-${VERSION}
    mkdir build && cd build
    cmake .. -DBUILD_SHARED_LIBS=ON \
             -DCMAKE_BUILD_TYPE=Release
    make -j${THREAD_NUM}
    make install
popd

ldconfig

# cleanup
rm -rf $PKG_NAME gflags-$VERSION

# Install glog which also depends on gflags.

VERSION="0.4.0"
PKG_NAME="glog-${VERSION}.tar.gz"
CHECKSUM="f28359aeba12f30d73d9e4711ef356dc842886968112162bc73002645139c39c"
DOWNLOAD_LINK="https://github.com/google/glog/archive/v${VERSION}.tar.gz"
# https://github.com/google/glog/archive/v0.4.0.tar.gz

download_if_not_cached "${PKG_NAME}" "${CHECKSUM}" "${DOWNLOAD_LINK}"

tar xzf ${PKG_NAME}

pushd glog-${VERSION}
    mkdir build && cd build
    cmake .. \
        -DBUILD_SHARED_LIBS=ON \
        -DWITH_GFLAGS=OFF \
        -DCMAKE_BUILD_TYPE=Release

    # if [ "$ARCH" == "aarch64" ]; then
    #    ./configure --build=armv8-none-linux --enable-shared
    # fi

    make -j${THREAD_NUM}
    make install
popd

ldconfig

# clean up.
rm -fr ${PKG_NAME} glog-${VERSION}